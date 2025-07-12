// (c) 2017 Blai Bonet

#ifndef SIM_PLANNER_H
#define SIM_PLANNER_H

#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "planner.h"
#include "node.h"
#include "screen.h"
#include "logger.h"
#include "utils.h"
#include "/usr/local/include/ale/ale_interface.hpp"
using namespace ale;
struct SimPlanner;
struct Sketch {
    std::function<bool(const SimPlanner&, const std::vector<pixel_t>&,const std::vector<pixel_t>&)> precondition;
    std::function<bool(const SimPlanner&, const std::vector<pixel_t>&, const std::vector<pixel_t>&, const std::vector<pixel_t>&)> goal;
    std::string description;
};

struct SimPlanner : Planner {
    ALEInterface &sim_;
    const int SCREENSHOT_HEIGHT = 600;
    const int SCREENSHOT_WIDTH = 800;
    // Screen dimensions (Atari standard screen size)
    const int SCREEN_HEIGHT = 210;
    const int SCREEN_WIDTH = 160;
    // Scaling factors based on the screenshot resolution
    const float SCALE_X = static_cast<float>(SCREEN_WIDTH) / SCREENSHOT_WIDTH;
    const float SCALE_Y = static_cast<float>(SCREEN_HEIGHT) / SCREENSHOT_HEIGHT;
    std::vector<Sketch> sketches_;
    std::vector<int> initial_screen_pixels_;
    mutable int priority_;
    const size_t frameskip_;
    const bool use_minimal_action_set_;
    const int simulator_budget_;
    const size_t num_tracked_atoms_;

    mutable size_t simulator_calls_;
    mutable float sim_time_;
    mutable float sim_reset_time_;
    mutable float sim_get_set_state_time_;

    mutable size_t get_atoms_calls_;
    mutable float get_atoms_time_;
    mutable float novel_atom_time_;
    mutable float update_novelty_time_;
    mutable bool ball_spawned_;
    ALEState initial_sim_state_;
    ActionVect action_set_;
    mutable bool printing_sketches_;
    const int lookahead_;
    SimPlanner(ALEInterface &sim,
               size_t frameskip,
               bool use_minimal_action_set,
               int simulator_budget,
               size_t num_tracked_atoms, int lookahead = 10, bool printing_sketche = false)
      : Planner(),
        sim_(sim),
        frameskip_(frameskip),
        use_minimal_action_set_(use_minimal_action_set),
        simulator_budget_(simulator_budget),
        num_tracked_atoms_(num_tracked_atoms), lookahead_(lookahead), printing_sketches_(printing_sketche)  {
        //static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 required");
        assert(sim_.getInt("frame_skip") == int(frameskip_));
        if( use_minimal_action_set_ )
            action_set_ = sim_.getMinimalActionSet();
        else
            action_set_ = sim_.getLegalActionSet();

        assert(sim_.getInt("frame_skip") == int(frameskip_));
        if (action_set_.size() > 6) {
            action_set_.resize(6);
            /*for (size_t i = 0; i < action_set_.size(); ++i) {
                std::cout << "Action " << i << ": " << action_set_[i] << std::endl;
            }*/
        }
        reset_game(sim_);
        get_state(sim_, initial_sim_state_);
        priority_ = 0; 
        //MyALEScreen screen(sim_, 3, &initial_screen_pixels_);
    }
    virtual ~SimPlanner() { }

    void reset_stats() const {
        simulator_calls_ = 0;
        sim_time_ = 0;
        sim_reset_time_ = 0;
        sim_get_set_state_time_ = 0;
        update_novelty_time_ = 0;
        get_atoms_calls_ = 0;
        get_atoms_time_ = 0;
        novel_atom_time_ = 0;

    }

    virtual float simulator_time() const {
        return sim_time_ + sim_reset_time_ + sim_get_set_state_time_;
    }
    virtual size_t simulator_calls() const {
        return simulator_calls_;
    }
    virtual Action random_action() const {
        return action_set_[lrand48() % action_set_.size()];
    }

    Action random_zero_value_action(const Node *root, float discount) const {
        assert(root != 0);
        assert((root->num_children_ > 0) && (root->first_child_ != nullptr));
        std::vector<Action> zero_value_actions;
        for( Node *child = root->first_child_; child != nullptr; child = child->sibling_ ) {
            if( child->qvalue(discount) == 0 )
                zero_value_actions.push_back(child->action_);
        }
        assert(!zero_value_actions.empty());
        return zero_value_actions[lrand48() % zero_value_actions.size()];
    }

    float call_simulator(ALEInterface &ale, Action action) const {
        ++simulator_calls_;
        float start_time = Utils::read_time_in_seconds();
        float reward = ale.act(action);
        assert(reward != -std::numeric_limits<float>::infinity());
        sim_time_ += Utils::read_time_in_seconds() - start_time;
        return reward;
    }

    void reset_game(ALEInterface &ale) const {
        float start_time = Utils::read_time_in_seconds();
        ale.reset_game();
        reset_planner_state(); 
        sim_reset_time_ += Utils::read_time_in_seconds() - start_time;
    }
    
    // Reset planner state on game reset
    void reset_planner_state() const {
        priority_ = 0;
        ydragon = false;
        gdragon = false;
        Last_room_color = 1;
    }

    void get_state(ALEInterface &ale, ALEState &ale_state) const {
        float start_time = Utils::read_time_in_seconds();
        ale_state = ale.cloneState();
        sim_get_set_state_time_ += Utils::read_time_in_seconds() - start_time;
    }
    void set_state(ALEInterface &ale, const ALEState &ale_state) const {
        float start_time = Utils::read_time_in_seconds();
        ale.restoreState(ale_state);
        sim_get_set_state_time_ += Utils::read_time_in_seconds() - start_time;
    }

    int get_lives(ALEInterface &ale) const {
        return ale.lives();
    }
    bool terminal_state(ALEInterface &ale) const {
        return ale.game_over();
    }

    const ALERAM& get_ram(ALEInterface &ale) const {
        return ale.getRAM();
    }
    void get_ram(ALEInterface &ale, std::string &ram_str) const {
        ram_str = std::string(256, '0');
        const ALERAM &ale_ram = get_ram(ale);
        for( size_t k = 0; k < 128; ++k ) {
            byte_t byte = ale_ram.get(k);
            ram_str[2 * k] = "01234567890abcdef"[byte >> 4];
            ram_str[2 * k + 1] = "01234567890abcdef"[byte & 0xF];
        }
    }

    // update info for node
    void update_info(Node *node, int screen_features, float alpha, bool use_alpha_to_update_reward_for_death) const {
        assert(node->is_info_valid_ != 2);
        assert(node->state_ == nullptr);
        assert(node->parent_ != nullptr);
        assert((node->parent_->is_info_valid_ == 1) || (node->parent_->state_ != nullptr));
        if( node->parent_->state_ == nullptr ) {
            // do recursion on parent
            update_info(node->parent_, screen_features, alpha, use_alpha_to_update_reward_for_death);
        }
        assert(node->parent_->state_ != nullptr);
        set_state(sim_, *node->parent_->state_);
        float reward = call_simulator(sim_, node->action_);
        assert(reward != std::numeric_limits<float>::infinity());
        assert(reward != -std::numeric_limits<float>::infinity());
        node->state_ = new ALEState;
        get_state(sim_, *node->state_);
        if( node->is_info_valid_ == 0 ) {
            node->reward_ = reward;
            node->terminal_ = terminal_state(sim_);
            if( node->reward_ < 0 ) node->reward_ *= alpha;
            get_atoms(node, screen_features);
            node->ale_lives_ = get_lives(sim_);
            if( use_alpha_to_update_reward_for_death && (node->parent_ != nullptr) && (node->parent_->ale_lives_ != -1) ) {
                if( node->ale_lives_ < node->parent_->ale_lives_ ) {
                    node->reward_ = -10 * alpha;
                    //logos_ << "L" << std::flush;
                }
            }
            node->path_reward_ = node->parent_ == nullptr ? 0 : node->parent_->path_reward_;
            node->path_reward_ += node->reward_;
        }
        node->grandfather = node->parent_->parent_;
        node->is_info_valid_ = 2;
    }

    // get atoms from ram or screen
    void get_atoms(const Node *node, int screen_features) const {
        assert(node->feature_atoms_.empty());
        ++get_atoms_calls_;
        if( screen_features == 0 ) { // RAM mode
            get_atoms_from_ram(node);
        } else {
            get_atoms_from_screen(node, screen_features);
            if( (node->parent_ != nullptr) && (node->parent_->feature_atoms_ == node->feature_atoms_) ) {
                node->frame_rep_ = node->parent_->frame_rep_ + frameskip_;
                assert((node->num_children_ == 0) && (node->first_child_ == nullptr));
            }
        }
        assert((node->frame_rep_ == 0) || (screen_features > 0));
    }
    void get_atoms_from_ram(const Node *node) const {
        assert(node->feature_atoms_.empty());
        node->feature_atoms_ = std::vector<int>(128, 0);
        float start_time = Utils::read_time_in_seconds();
        const ALERAM &ram = get_ram(sim_);
        for( size_t k = 0; k < 128; ++k ) {
            node->feature_atoms_[k] = (k << 8) + ram.get(k);
            assert((k == 0) || (node->feature_atoms_[k] > node->feature_atoms_[k-1]));
        }
        get_atoms_time_ += Utils::read_time_in_seconds() - start_time;
    }
    void get_atoms_from_screen(const Node *node, int screen_features) const {
        assert(node->feature_atoms_.empty());
        float start_time = Utils::read_time_in_seconds();
        if( (screen_features < 3) || (node->parent_ == nullptr) ) {
            MyALEScreen screen(sim_, screen_features, &node->feature_atoms_, &node->screen_pixels_);
        }
        else {
            assert((screen_features >= 3) && (node->parent_ != nullptr));
            MyALEScreen screen(sim_, screen_features, &node->feature_atoms_, &node->screen_pixels_ ,&node->parent_->feature_atoms_);
        }
        get_atoms_time_ += Utils::read_time_in_seconds() - start_time;
    }

    // novelty tables: a (simple) novelty table maps feature indices to best depth at which
    // features have been seen. Best depth is initialized to max.int. Novelty table associated
    // to node is a unique simple table if subtables is disabled. Otherwise, there is one table
    // for each different logscore. The table for a node is the table for its logscore.
    int logscore(float path_reward) const {
        if( path_reward <= 0 ) {
            return 0;
        } else {
            int logr = int(floorf(log2f(path_reward)));
            return path_reward < 1 ? logr : 1 + logr;
        }
    }
    int get_index_for_novelty_table(const Node *node, bool use_novelty_subtables) const {
        return !use_novelty_subtables ? 0 : logscore(node->path_reward_);
    }

    std::vector<int>& get_novelty_table(const Node *node, std::map<int, std::vector<int> > &novelty_table_map, bool use_novelty_subtables) const {
        int index = get_index_for_novelty_table(node, use_novelty_subtables);
        std::map<int, std::vector<int> >::iterator it = novelty_table_map.find(index);
        if( it == novelty_table_map.end() ) {
            novelty_table_map.insert(std::make_pair(index, std::vector<int>()));
            std::vector<int> &novelty_table = novelty_table_map.at(index);
            novelty_table = std::vector<int>(num_tracked_atoms_, std::numeric_limits<int>::max());
            return novelty_table;
        } else {
            return it->second;
        }
    }

    size_t update_novelty_table(size_t depth, const std::vector<int> &feature_atoms, std::vector<int> &novelty_table) const {
        float start_time = Utils::read_time_in_seconds();
        size_t first_index = 0;
        size_t number_updated_entries = 0;
        for( size_t k = first_index; k < feature_atoms.size(); ++k ) {
            assert((feature_atoms[k] >= 0) && (feature_atoms[k] < int(novelty_table.size())));
            if( int(depth) < novelty_table[feature_atoms[k]] ) {
                novelty_table[feature_atoms[k]] = depth;
                ++number_updated_entries;
            }
        }
        update_novelty_time_ += Utils::read_time_in_seconds() - start_time;
        return number_updated_entries;
    }

    int get_novel_atom(size_t depth, const std::vector<int> &feature_atoms, const std::vector<int> &novelty_table) const {
        float start_time = Utils::read_time_in_seconds();
        for( size_t k = 0; k < feature_atoms.size(); ++k ) {
            assert(feature_atoms[k] < int(novelty_table.size()));
            if( novelty_table[feature_atoms[k]] > int(depth) ) {
                novel_atom_time_ += Utils::read_time_in_seconds() - start_time;
                return feature_atoms[k];
            }
        }
        for( size_t k = 0; k < feature_atoms.size(); ++k ) {
            if( novelty_table[feature_atoms[k]] == int(depth) ) {
                novel_atom_time_ += Utils::read_time_in_seconds() - start_time;
                return feature_atoms[k];
            }
        }
        novel_atom_time_ += Utils::read_time_in_seconds() - start_time;
        assert(novelty_table[feature_atoms[0]] < int(depth));
        return feature_atoms[0];
    }

    size_t num_entries(const std::vector<int> &novelty_table) const {
        assert(novelty_table.size() == num_tracked_atoms_);
        size_t n = 0;
        for( size_t k = 0; k < novelty_table.size(); ++k )
            n += novelty_table[k] < std::numeric_limits<int>::max();
        return n;
    }

    // prefix
    void apply_prefix(ALEInterface &ale, const ALEState &initial_state, const std::vector<Action> &prefix, ALEState *last_state = nullptr) const {
        assert(!prefix.empty());
        reset_game(ale);
        set_state(ale, initial_state);
        for( size_t k = 0; k < prefix.size(); ++k ) {
            if( (last_state != nullptr) && (1 + k == prefix.size()) )
                get_state(ale, *last_state);
            call_simulator(ale, prefix[k]);
        }
    }

    void print_prefix(logging::Logger::mode_t logger_mode, const std::vector<Action> &prefix) const {
        logging::Logger::Continuation(logger_mode) << "[";
        for( size_t k = 0; k < prefix.size(); ++k )
            logging::Logger::Continuation(logger_mode) << prefix[k] << ",";
        logging::Logger::Continuation(logger_mode) << "]" << std::flush;
    }

    // generate states along given branch
    void generate_states_along_branch(Node *node,
                                      const std::deque<Action> &branch,
                                      int screen_features,
                                      float alpha,
                                      bool use_alpha_to_update_reward_for_death) const {
        for( size_t pos = 0; pos < branch.size(); ++pos ) {
            if( node->state_ == nullptr ) {
                assert(node->is_info_valid_ == 1);
                update_info(node, screen_features, alpha, use_alpha_to_update_reward_for_death);
            }

            Node *selected = nullptr;
            for( Node *child = node->first_child_; child != nullptr; child = child->sibling_ ) {
                if( child->action_ == branch[pos] ) {
                    selected = child;
                    break;
                }
            }
            assert(selected != nullptr);
            node = selected;
        }
    }
    pixel_t greyscale(int r, int g, int b) {
        return static_cast<pixel_t>(r * 0.3 + g * 0.59 + b * 0.11);
    }
   
    //adventure
    mutable int Last_room_color = 1; // 0 yellow throne, 1 yellow 2 green 3 purpel 4 red 5 light green 6 blue 7 black 8 red 9 pink 
    mutable bool ydragon = false;
    mutable bool gdragon = false;
    // Navigation state variables
    mutable bool reachednav1 = false;
    mutable bool reachednav2 = false;
    mutable bool reachednav3 = false;
    mutable bool second_6 = false; 
    mutable bool ykeyt  = false;
    mutable bool bkeyt = false;
    mutable bool yswrt = false; 
    mutable bool chalicet = false;
    const bool printing_debug = false; // Set to true to enable debug printing
    const bool impotant_debug = true;
    const bool printing_debug_adventure = false; // Set to true to enable debug printing for adventure mode
    const std::map<std::string, pixel_t> COLORS = {
        {"yellow", 193},
        {"blue", 85},
        {"red", 129},
        {"black", 0},
        {"grey", 170},
        {"green", 147},
        {"purple", 157},
        {"light_green", 157},
        {"pink", 107},
        {"white", 255},
        {"gdragon", 147},
        {"ydragon", 193}
        };

    const int adventure_cube_width = 4; // Width of the adventure cube in pixels
    const int adventure_cube_height = 8; // Height of the adventure cube in pixels
    mutable int count =0 ; 
    const pixel_t COLOR_THRESHOLD = 5;
    
     //adventure
     // Helper functions
     int color_diff(pixel_t c1, pixel_t c2) const {
        return std::abs(static_cast<int>(c1) - static_cast<int>(c2));
     }
    int manhattan_dist(int x1, int y1, int x2, int y2) const {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }
    bool color_match(pixel_t c1, pixel_t c2) const {return color_diff(c1, c2) <= COLOR_THRESHOLD;}
    bool is_grey(pixel_t px) const {return color_match(px, COLORS.at(std::string("grey")));}
    bool check_surrounding_grey(const std::vector<pixel_t>& screen_pixels, size_t x, size_t y) const {
       bool left = (x-1 >= 0) && is_grey(screen_pixels[y * SCREEN_WIDTH + (x - 1)]);
       bool right = x+ adventure_cube_width+1 < SCREEN_WIDTH && is_grey(screen_pixels[y * SCREEN_WIDTH + (x + adventure_cube_width+1)]);
       bool up =( y-1 >= 0) && is_grey(screen_pixels[(y - 1) * SCREEN_WIDTH + x]);
       bool down = y+ adventure_cube_height+1 < SCREEN_HEIGHT && is_grey(screen_pixels[(y + adventure_cube_height+1) * SCREEN_WIDTH + x]);
       bool is_black = color_match(COLORS.at(std::string("black")), screen_pixels[5*SCREEN_WIDTH +5 ]); 
       if (is_black){
        left = (x-1 >= 0) &&  !color_match(COLORS.at(std::string("black")), screen_pixels[y * SCREEN_WIDTH + (x - 1)]);
        right = x+ adventure_cube_width+1 < SCREEN_WIDTH && !color_match(COLORS.at(std::string("black")),screen_pixels[y * SCREEN_WIDTH + (x + adventure_cube_width+1)]);
        up = (y-1 >= 0) && !color_match(COLORS.at(std::string("black")),screen_pixels[(y - 1) * SCREEN_WIDTH + x]);
        down = y+ adventure_cube_height+1 < SCREEN_HEIGHT && !color_match(COLORS.at(std::string("black")),screen_pixels[(y + adventure_cube_height+1) * SCREEN_WIDTH + x]); 
       }
       return (left || right) && ( up || down); 
    }
    bool is_key_area(const std::vector<pixel_t>& screen_pixels, int x, int y) const {
    int count = 0;
    for (int dy = 0; dy < 2; dy++) {
        for (int dx = 0; dx < 1; dx++) {
            int px = x + 2 + dx;
            int py = y + 4 + dy;
            if (px < 0 || px >= SCREEN_WIDTH || py < 0 || py >= SCREEN_HEIGHT) 
                continue;
            size_t index = py * SCREEN_WIDTH + px;
            if (is_grey(screen_pixels[index])) 
                count++;
        }
    }
    return count >= 2;
}
    int calculate_distance_from_goal(const std::vector<pixel_t>& screen_pixels) const {
        pixel_t current_room_color = screen_pixels[SCREEN_WIDTH * 5 + 5]; 
       
        pixel_t special_yellow_case = screen_pixels[SCREEN_WIDTH * 80 + 80]; //82 80
        //mutable int Last_room_color = 1; // 0 yellow throne, 1 yellow 2 green 3 purpel 4 red 5 light green 6 blue 7 black 8 red 9 pink 
        if (color_match(current_room_color, COLORS.at("yellow"))) {
            Last_room_color = 1;
            if (!color_match(special_yellow_case, COLORS.at("yellow"))) {
                Last_room_color = 0;

            }
        } else if ( color_match(current_room_color, COLORS.at("green"))) Last_room_color = 2; 
        else if (color_match(current_room_color, COLORS.at("purple"))) Last_room_color = 3;  
        else if (color_match(current_room_color, COLORS.at("light_green"))) Last_room_color = 5; 
        else if (color_match(current_room_color, COLORS.at("blue"))) Last_room_color = 6; 
        else if (color_match(current_room_color, COLORS.at("black"))) {
            Last_room_color = 7;
            // Reset navigation states when entering black throne
            reachednav1 = false;
            reachednav2 = false;
            reachednav3 = false;
        } 
        else if (color_match(current_room_color, COLORS.at("pink"))) Last_room_color = 9; 
        else if (color_match(current_room_color, COLORS.at("red"))) {
            if (Last_room_color == 3) {
                Last_room_color = 4; // Special case for red in the yellow room
            } else if (Last_room_color == 7 || Last_room_color == 9) {
                Last_room_color = 8; // Special case for red in the purple room
                
            } else {
                Last_room_color = -1; // General case for red
            }
        } else {
            
             if (printing_debug)std::cout << "Unknown room color: " << static_cast<int>(current_room_color) << std::endl;
            Last_room_color = -1; // Reset to unknown if color does not match any known col
        }
        return Last_room_color;
    }
    
    std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions_for_cube(const std::vector<pixel_t>& screen_pixels) const {
        std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions;

        // Get key colors from the screen
        pixel_t cube_color = screen_pixels[5 * SCREEN_WIDTH + 5];
        bool is_yellow = color_match(cube_color, COLORS.at("yellow"));
        bool is_black = color_match(cube_color, COLORS.at("black"));
        bool is_red = color_match(cube_color, COLORS.at("red"));
        bool is_pink = color_match(cube_color, COLORS.at("pink"));
        bool is_blue = color_match(cube_color, COLORS.at("blue"));
        bool is_green = color_match(cube_color, COLORS.at("green"));
        bool is_light_green = color_match(cube_color, COLORS.at("light_green"));
        bool is_purple = color_match(cube_color, COLORS.at("purple"));

        // Helper lambda for color_match with screen_pixels
        auto color_match_at = [&](int x, int y, const std::string& color) {
            size_t idx = static_cast<size_t>(y) * SCREEN_WIDTH + static_cast<size_t>(x);
            return color_match(screen_pixels[idx], COLORS.at(color));
        };
        if(is_black){
            // Black throne room
                regions.push_back({{7, 18}, {40, 178}});
                regions.push_back({{119, 18}, {151, 178}});
                regions.push_back({{7, 82}, {47, 178}});
                regions.push_back({{111, 82}, {151, 178}});
                regions.push_back({{7, 146}, {151, 178}});
                regions.push_back({{63, 146}, {96, 195}});
        } else if (is_yellow && color_match_at(80, 80, "yellow")) {
                // Yellow throne room
                regions.push_back({{7, 18}, {40, 178}});
                regions.push_back({{119, 18}, {152, 178}});
                regions.push_back({{7, 82}, {48, 146}});
                regions.push_back({{111, 82}, {151, 146}});
                regions.push_back({{7, 147}, {152, 178}});
                //std::cout<<"yellow_throne_room"<<std::endl; 
        } else if(is_yellow) {
                // Normal yellow/black room
                regions.push_back({{7, 18}, {152, 179}});
        }else if (is_red || is_pink) {
            regions.push_back({{7, 18}, {152, 179}});
        } 
        else if (is_green || is_light_green || is_purple) {
            if (is_light_green) {
                regions.push_back({{12, 18}, {160, 178}});  // x >= 12
            } else if (is_purple) {
                regions.push_back({{0, 18}, {147, 178}});   // x <= 147
            }else{
                regions.push_back({{0, 18}, {160, 178}});
            }
        } 
        
        else if (is_blue) {
            // Determine blue room type
            if (color_match_at(79, 6, "blue")) {
                // Blue room 1
                regions.push_back({{0, 19}, {24, 50}});
                regions.push_back({{134, 19}, {160, 50}});
                regions.push_back({{15, 50}, {24, 83}});
                regions.push_back({{135, 50}, {144, 83}});
                regions.push_back({{0, 83}, {24, 114}});
                regions.push_back({{31, 1}, {40, 178}});
                regions.push_back({{119, 1}, {128, 178}});
                regions.push_back({{0, 148}, {160, 178}});
                regions.push_back({{47, 1}, {55, 114}});
                regions.push_back({{103, 1}, {112, 114}});
                regions.push_back({{47, 83}, {111, 114}});
                regions.push_back({{63, 1}, {72, 50}});
                regions.push_back({{87, 1}, {96, 50}});
                regions.push_back({{63, 18}, {96, 50}});
            } 
            else if (color_match_at(77, 185, "blue")) {
                // Blue room 4
                regions.push_back({{0, 146}, {23, 178}});
                regions.push_back({{135, 146}, {160, 178}});
                regions.push_back({{0, 18}, {23, 50}});
                regions.push_back({{135, 18}, {160, 50}});
                regions.push_back({{15, 18}, {23, 114}});
                regions.push_back({{135, 18}, {144, 114}});
                regions.push_back({{15, 83}, {144, 114}});
                regions.push_back({{31, 83}, {127, 178}});
                regions.push_back({{31, 1}, {40, 50}});
                regions.push_back({{103, 1}, {112, 50}});
                regions.push_back({{119, 1}, {127, 50}});
                regions.push_back({{47, 1}, {56, 50}});
                regions.push_back({{103, 19}, {127, 50}});
                regions.push_back({{31, 19}, {56, 50}});
            } 
            else if (color_match_at(20, 8, "blue")) {
                // Blue room 3
                regions.push_back({{0, 19}, {31, 50}});
                regions.push_back({{128, 19}, {160, 50}});
                regions.push_back({{15, 50}, {31, 114}});
                regions.push_back({{128, 50}, {143, 114}});
                regions.push_back({{0, 147}, {23, 178}});
                regions.push_back({{136, 147}, {160, 178}});
                regions.push_back({{15, 147}, {23, 194}});
                regions.push_back({{136, 147}, {143, 194}});
                regions.push_back({{31, 146}, {63, 178}});
                regions.push_back({{31, 146}, {39, 194}});
                regions.push_back({{96, 146}, {127, 178}});
                regions.push_back({{63, 1}, {96, 51}});
                regions.push_back({{72, 1}, {88, 195}});
                regions.push_back({{39, 18}, {56, 114}});
                regions.push_back({{103, 18}, {120, 114}});
                regions.push_back({{39, 82}, {120, 114}});
            } 
            else {
                // Replace blue room type 2 region definitions with:
                regions.push_back({{15, 1}, {23, 50}});
                regions.push_back({{135, 1}, {143, 50}});
                regions.push_back({{0, 18}, {23, 50}});
                regions.push_back({{135, 18}, {160, 50}});
                regions.push_back({{0, 83}, {39, 114}});
                regions.push_back({{120, 83}, {160, 114}});
                regions.push_back({{0, 147}, {23, 178}});
                regions.push_back({{135, 147}, {160, 178}});
                regions.push_back({{16, 83}, {23, 178}});
                regions.push_back({{136, 83}, {144, 178}});
                regions.push_back({{31, 83}, {40, 195}});
                regions.push_back({{120, 83}, {128, 195}});
                regions.push_back({{119, 1}, {127, 50}});
                regions.push_back({{31, 1}, {40, 50}});
                regions.push_back({{103, 19}, {127, 50}});
                regions.push_back({{31, 19}, {55, 50}});
                regions.push_back({{103, 50}, {111, 194}});
                regions.push_back({{47, 50}, {55, 194}});
                regions.push_back({{72, 1}, {87, 194}});
                regions.push_back({{63, 146}, {95, 194}});
            }
        }else{
            if(printing_debug) std::cout << "No suitable room region for cube detection. Cube color: " << static_cast<int>(cube_color) << std::endl;
            
            //std::cout << " no suitable room region"; 
            regions.push_back({{0, 0}, {160, 210}});
        }

        // Add entrance areas
        std::pair<std::pair<int,int>, std::pair<int,int>> entrance_down = {{63, 170}, {96, 195}};
        std::pair<std::pair<int,int>, std::pair<int,int>> entrance_up = {{63, 1}, {96, 26}};
        
        
        if (is_green || is_light_green || is_red||is_pink) {
            regions.push_back(entrance_up);
        }else{
            regions.push_back(entrance_down);
        }
       
        // Special case for red room when coming from black or pink
        if (is_red && (Last_room_color == 7 || Last_room_color == 9)) {
            regions.push_back(entrance_down);
        }

        return regions;
    }
    
    bool item_surroundin_grey(const std::vector<pixel_t>& screen_pixels, size_t x, size_t y) const {
            bool left = x-5 >= 0 && is_grey(screen_pixels[y * SCREEN_WIDTH + (x -5 )]);
            bool right = x+ 5 < SCREEN_WIDTH && is_grey(screen_pixels[y * SCREEN_WIDTH + (x + 5)]);
            bool up = y-5 >= 0 && is_grey(screen_pixels[(y - 5) * SCREEN_WIDTH + x]);
            bool down = y+ 5 < SCREEN_HEIGHT && is_grey(screen_pixels[(y + 5) * SCREEN_WIDTH + x]);
            return (left || right) && ( up || down); 
        }
        
    std::pair<int,int> find_cube_without_reference(const std::vector<pixel_t>& current) const{
        std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions = regions_for_cube(current);
        std::pair<int,int> cube_position = {-1, -1}; // Default position if no cube is found
        for (const auto& region : regions) {
            int x1 = region.first.first;
            int y1 = region.first.second;
            int x2 = region.second.first;
            int y2 = region.second.second;
            cube_position = find_cube_candidates(current, x1, y1, x2, y2);
            if( cube_position.first != -1 && cube_position.second != -1) {
                return cube_position; // Return the first found cube position
            }

        }
        return cube_position; // Return the default position if no cube is found
    }
    int count_matching_pixels(const std::vector<pixel_t>& screen_pixels, int x, int y, pixel_t cube_color) const {
        int count = 0;
        for (int dy = 0; dy < adventure_cube_height; ++dy) {
            for (int dx = 0; dx < adventure_cube_width; ++dx) {
                int px = x + dx;
                int py = y + dy;
                if (px < 0 || px >= SCREEN_WIDTH || py < 0 || py >= SCREEN_HEIGHT) {
                    continue;
                }
                size_t idx = py * SCREEN_WIDTH + px;
                if (color_match(screen_pixels[idx], cube_color)) {
                    count++;
                }
            }
        }
        return count;
    }

    std::pair<int, int> find_cube_candidates(const std::vector<pixel_t>& screen_pixels, int x1, int y1, int x2, int y2) const {
        pixel_t cube_color = screen_pixels[SCREEN_WIDTH * 5+5];
        bool is_blue = color_match(cube_color, COLORS.at("blue"));
        std::vector<std::pair<int, int>> candidates;

        for (int y = y1; y <= y2 - adventure_cube_height; ++y) {
            for (int x = x1; x <= x2 - adventure_cube_width; ++x) {
                int cx = x + adventure_cube_width / 2;
                int cy = y + adventure_cube_height / 2;
                
                // Check center and surrounding pixels
                bool valid = true;
                std::vector<std::pair<int, int>> points = {
                    {cx, cy},  // Center
                    {cx - adventure_cube_width/4, cy},  // Left
                    {cx + adventure_cube_width/4, cy},  // Right
                    {cx, cy - adventure_cube_height/4}, // Top
                    {cx, cy + adventure_cube_height/4}  // Bottom
                };
                
                for (const auto& pt : points) {
                    if (pt.first < 0 || pt.first >= SCREEN_WIDTH || 
                        pt.second < 0 || pt.second >= SCREEN_HEIGHT) {
                        valid = false;
                        break;
                    }
                    
                    size_t idx = pt.second * SCREEN_WIDTH + pt.first;
                    if (!color_match(screen_pixels[idx], cube_color)) {
                        valid = false;
                        break;
                    }
                }
                
                if (!valid) continue;
                if (!check_surrounding_grey(screen_pixels, x, y)) continue; // Updated
                candidates.push_back({x, y});  // Store candidate
            }
        }

        // Select best candidate based on pixel density
        if (candidates.empty()) {
             //std::cout<< "candidates empty" << std::endl;
            return {-1, -1};
        }

        int best_count = -1;
        std::pair<int, int> best_candidate = {-1, -1};
        for (const auto& cand : candidates) {
            int count = count_matching_pixels(screen_pixels, cand.first, cand.second, cube_color);
            if (count > best_count) {
                best_count = count;
                best_candidate = cand;
            }
        }
        if (best_candidate.first == -1){
             //std::cout<< "no best candidate" << std::endl;
        }
        if(printing_debug) std::cout << "Best candidate: (" << best_candidate.first << ", " << best_candidate.second << ") with count: " << best_count << std::endl;
        //std::cout << "cube found: " << best_candidate.first << " " << best_candidate.second << std::endl;
        return best_candidate;
    }
    
    std::pair<int,int> highlight_cube(const std::vector<pixel_t>& current, const std::vector<pixel_t>& prev  )  const{
        std::pair<int,int> temp = {-1,-1};
        temp = find_cube_without_reference(current); 
        
        return temp;
    }
        // Helper function to get cube center coordinates
    std::pair<int, int> get_cube_center(const std::vector<pixel_t>& screen_pixels) const {
        auto cube_coords = find_cube_without_reference(screen_pixels);
        if (cube_coords.first == -1) return {-1, -1};
        return {
            cube_coords.first + adventure_cube_width / 2,
            cube_coords.second + adventure_cube_height / 2
        };
    }   
    
    std::vector<std::set<std::pair<int, int>>> cluster_pixels(const std::set<std::pair<int, int>>& candidates,const std::vector<pixel_t>& screen_pixels) const {
        std::vector<std::set<std::pair<int, int>>> clusters;
        std::set<std::pair<int, int>> visited;
        const std::vector<std::pair<int, int>> dirs = {{1,0}, {-1,0}, {0,1}, {0,-1}};

        for (const auto& pixel : candidates) {
            if (visited.find(pixel) != visited.end()) continue;

            std::queue<std::pair<int, int>> queue;
            std::set<std::pair<int, int>> cluster;
            queue.push(pixel);
            visited.insert(pixel);
            cluster.insert(pixel);
            pixel_t base_color = screen_pixels[pixel.second * SCREEN_WIDTH + pixel.first];

            while (!queue.empty()) {
                auto [x, y] = queue.front();
                queue.pop();

                for (const auto& [dx, dy] : dirs) {
                    int nx = x + dx;
                    int ny = y + dy;
                    std::pair<int, int> neighbor = {nx, ny};
                    
                    // Check bounds and if already visited
                    if (nx < 0 || nx >= SCREEN_WIDTH || 
                        ny < 0 || ny >= SCREEN_HEIGHT) continue;
                    if (visited.find(neighbor) != visited.end()) continue;
                    
                    // Check color match and candidate status
                    pixel_t neighbor_color = screen_pixels[ny * SCREEN_WIDTH + nx];
                    if (color_match(neighbor_color, base_color) &&
                        candidates.find(neighbor) != candidates.end()) {
                        visited.insert(neighbor);
                        cluster.insert(neighbor);
                        queue.push(neighbor);
                    }
                }
            }
            
            if (!cluster.empty()) {
                clusters.push_back(cluster);
            }
        }
        return clusters;
    }
    
    std::set<std::pair<int, int>> form_cluster_from_seed(const std::vector<pixel_t>& screen_pixels,std::pair<int, int> seed,std::set<std::pair<int, int>>& visited,const std::set<std::pair<int, int>>& candidate_set) const {
        std::set<std::pair<int, int>> cluster;
        const std::vector<std::pair<int, int>> directions = {{1,0}, {-1,0}, {0,1}, {0,-1}};
        std::queue<std::pair<int, int>> queue;
        
        pixel_t base_color = screen_pixels[seed.second * SCREEN_WIDTH + seed.first];
        queue.push(seed);
        visited.insert(seed);
        cluster.insert(seed);
        
        while (!queue.empty()) {
            auto [cx, cy] = queue.front();
            queue.pop();
            
            for (const auto& [dx, dy] : directions) {
                int nx = cx + dx;
                int ny = cy + dy;
                std::pair<int, int> neighbor = {nx, ny};
                
                // Only consider neighbors in candidate set
                if (candidate_set.find(neighbor) == candidate_set.end()) {
                    continue;
                }
                    
                // Check boundaries and visit status
                if (0 <= nx && nx < SCREEN_WIDTH && 
                    0 <= ny && ny < SCREEN_HEIGHT &&
                    visited.find(neighbor) == visited.end()) 
                {
                    pixel_t neighbor_color = screen_pixels[ny * SCREEN_WIDTH + nx];
                    if (color_match(neighbor_color, base_color)) {
                        visited.insert(neighbor);
                        queue.push(neighbor);
                        cluster.insert(neighbor);
                    }
                }
            }
        }
        if(printing_debug) std::cout << "clusters formed: " << cluster.size() << std::endl; 
        return cluster;
    }

    std::vector<std::set<std::pair<int, int>>> cluster_pixels_using_seed(const std::set<std::pair<int, int>>& candidates, const std::vector<pixel_t>& screen_pixels) const 
    {
        std::vector<std::set<std::pair<int, int>>> clusters;
        std::set<std::pair<int, int>> visited;
        
        for (const auto& pixel : candidates) {
            if (visited.find(pixel) != visited.end()) continue;
            
            auto cluster = form_cluster_from_seed(screen_pixels, pixel, visited, candidates);
            if (!cluster.empty()) {
                clusters.push_back(cluster);
            }
        }
        return clusters;
    }

    bool cluster_in_regions(const std::set<std::pair<int, int>>& cluster, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& regions) const {
        if (cluster.empty()) return false;

        int inside_count = 0;
        for (const auto& [x, y] : cluster) {
            for (const auto& region : regions) {
                auto [x1, y1] = region.first;
                auto [x2, y2] = region.second;
                if (x >= x1 && x <= x2 && y >= y1 && y <= y2) {
                    inside_count++;
                    break;
                }
            }
        }
        return (static_cast<float>(inside_count) / static_cast<float>(cluster.size()) )>= 0.5f;
    }

    // NEW HELPER: Filter clusters by exclusion points
    std::vector<std::set<std::pair<int, int>>> filter_clusters_by_exclusion_points(const std::vector<std::set<std::pair<int, int>>>& clusters,const std::set<std::pair<int, int>>& exclusion_points) const 
    {
        std::vector<std::set<std::pair<int, int>>> filtered_clusters;
        for (const auto& cluster : clusters) {
            bool contains_exclusion = false;
            for (const auto& pt : exclusion_points) {
                if (cluster.find(pt) != cluster.end()) {
                    contains_exclusion = true;
                    break;
                }
            }
            if (!contains_exclusion) {
                filtered_clusters.push_back(cluster);
            }
        }
        return filtered_clusters;
    }
    
    // Detect items in the entire screen
    std::vector<std::pair<std::string, std::pair<int, int>>> detect_items_entire_screen(const std::vector<pixel_t>& screen_pixels) const {
        std::vector<std::pair<std::string, std::pair<int, int>>> detected_items;
        auto regions = regions_for_cube(screen_pixels);
        auto cube_coords = find_cube_without_reference(screen_pixels);

        // Collect candidate pixels (non-grey) in entire screen
        std::set<std::pair<int, int>> candidates;
            for (int y = 0; y < SCREEN_HEIGHT; y++) {
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                // Skip cube area if found
                if (cube_coords.first != -1 && 
                    x >= cube_coords.first && x < cube_coords.first + adventure_cube_width &&
                    y >= cube_coords.second && y < cube_coords.second + adventure_cube_height) {
                    continue;
                }
                
                pixel_t px = screen_pixels[y * SCREEN_WIDTH + x];
                if (!is_grey(px)) {
                    candidates.insert({x, y});
                }
            }
        }

        // Cluster candidate pixels
        auto clusters = cluster_pixels(candidates, screen_pixels);

        // Analyze filtered clusters
        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;
            
            // Check if cluster is in valid regions
            if (!cluster_in_regions(cluster, regions)) continue;

            // Calculate cluster centroid
            int sum_x = 0, sum_y = 0;
            for (const auto& [x, y] : cluster) {
                sum_x += x;
                sum_y += y;
            }
            int center_x = sum_x / cluster.size();
            int center_y = sum_y / cluster.size();
            
            // Calculate cluster size and color
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];

            // Identify item type
            std::string item_type;
           if (size >= 26 && size <= 30 && color_match(color, COLORS.at("yellow"))) {
                item_type = "yellow_key";
            }else if (size >= 20 && size <= 25 && color_match(color, COLORS.at("yellow"))) {
                item_type = "yellow_sword";
            }
            else if (size >= 26 && size <= 30 && color_match(color, COLORS.at("black"))) {
                item_type = "black_key";
            } else if (size >= 67 && size <= 68) {
                item_type = "chalice";
            }
            
            if (!item_type.empty()) {
                detected_items.push_back({item_type, {center_x, center_y}});
            }
        } 
        /* std::cout<<std::endl<<"Detected items entire screen" << std::endl; 
        for (auto i : detected_items){
            std::cout << i.first << " at " << i.second.first << " " << i.second.second << std::endl; 
        }*/
        return detected_items;
    }
    
    // Detect items near cube position
    std::vector<std::pair<std::string, std::pair<int, int>>> detect_items_around_cube(const std::vector<pixel_t>& screen_pixels, const std::pair<int, int>& cube_pos) const {
        if (cube_pos.first == -1 || cube_pos.second == -1) {
            return {};
        }
        
        const int padding = 20;
        int x0 = cube_pos.first;
        int y0 = cube_pos.second;
        int x_start = std::max(0, x0 - padding);
        int y_start = std::max(0, y0 - padding);
        int x_end = std::min(SCREEN_WIDTH, x0 + adventure_cube_width + padding);
        int y_end = std::min(SCREEN_HEIGHT, y0 + adventure_cube_height + padding);

        // Collect candidate pixels (non-grey)
        std::set<std::pair<int, int>> candidates;
        for (int y = y_start; y < y_end; y++) {
            for (int x = x_start; x < x_end; x++) {
                pixel_t px = screen_pixels[y * SCREEN_WIDTH + x];
                if (!is_grey(px)) {
                    candidates.insert({x, y});
                }
            }
        }

        // Cluster candidate pixels
        auto clusters = cluster_pixels(candidates, screen_pixels);
        std::vector<std::pair<std::string, std::pair<int, int>>> items;
        
        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;
            
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            
            std::string item_type;
            if (size >= 26 && size <= 30) {
                if (color_match(color, COLORS.at("yellow"))) {
                    item_type = "yellow_key";
                } 
                else if (color_match(color, COLORS.at("black"))) {
                    item_type = "black_key";
                }
            } 
            else if (size >= 20 && size <= 25 && color_match(color, COLORS.at("yellow"))) {
                item_type = "yellow_sword";
            } 
            else if (size >= 67 && size <= 68) {
                item_type = "chalice";
            }
            
            if (!item_type.empty()) {
                // Calculate centroid
                int sum_x = 0, sum_y = 0;
                for (const auto& pt : cluster) {
                    sum_x += pt.first;
                    sum_y += pt.second;
                }
                int center_x = sum_x / cluster.size();
                int center_y = sum_y / cluster.size();
                items.push_back({item_type, {center_x, center_y}});
            }
        }
        
        return items;
    }
    
    // Dragon detection function
    void detect_dragons(const std::vector<pixel_t>& screen_pixels, std::string dragon_type) const {
        auto regions = regions_for_cube(screen_pixels);
        //update needed
        auto cube_coords = highlight_cube(screen_pixels,screen_pixels);
        
        // Get all items to exclude (entire screen)
        auto entire_items = detect_items_entire_screen(screen_pixels);

        std::vector<std::pair<std::string, std::pair<int, int>>> all_items;
            all_items.insert(all_items.end(), entire_items.begin(), entire_items.end());
        
            // Build exclusion set: cube area and item areas
            std::set<std::pair<int, int>> exclusion_set;
        
        // Add cube pixels
        if (cube_coords.first != -1) {
            for (int dy = 0; dy < adventure_cube_height; ++dy) {
                for (int dx = 0; dx < adventure_cube_width; ++dx) {
                    int x = cube_coords.first + dx;
                    int y = cube_coords.second + dy;
                    if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
                        exclusion_set.insert({x, y});
                    }
                }
            }
        }
        
        // Collect candidate pixels (non-grey in regions, not excluded)
        std::set<std::pair<int, int>> candidates;
        for (const auto& region : regions) {
            int x1 = region.first.first;
            int y1 = region.first.second;
            int x2 = region.second.first;
            int y2 = region.second.second;
            for (int y = y1; y <= y2; ++y) {
                for (int x = x1; x <= x2; ++x) {
                    if (exclusion_set.find({x, y}) != exclusion_set.end()) continue;
                    pixel_t px = screen_pixels[y * SCREEN_WIDTH + x];
                    if ( color_match(px, COLORS.at("yellow")) || color_match(px, COLORS.at("green"))) {
                        candidates.insert({x, y});
                    }
                }
            }
        }
        
        // Cluster and analyze
        auto clusters = cluster_pixels(candidates, screen_pixels);
        if(printing_debug) std::cout<<"cluster size" << clusters.size(); 
        // NEW: Filter clusters  using item centroids
        std::set<std::pair<int, int>> exclusion_centroids;
        for (const auto& item : all_items) {
            exclusion_centroids.insert(item.second);
        }
        auto filtered_clusters = clusters; 
        if(printing_debug) std::cout<< "the size of exlcusion points" << exclusion_centroids.size();
        if(!exclusion_centroids.empty())  filtered_clusters = filter_clusters_by_exclusion_points(clusters, exclusion_centroids);
        if(printing_debug) std::cout<<"cluster size after filtering" << filtered_clusters.size(); 
        for (const auto& cluster : filtered_clusters) {
            if (cluster.size() < 140 || cluster.size() >= 180) continue; // Dragon size threshold
            
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            
            std::string dragon_types;
            if (dragon_type == "gdragon" && color_match(color, COLORS.at("gdragon"))) {
                dragon_types = "gdragon";
            } else if (dragon_type == "ydragon" && color_match(color, COLORS.at("ydragon"))) {
                dragon_types = "ydragon";
            } else {
                continue;
            }
            
            // Determine state by size
            size_t size = cluster.size();
            if (size >= 140 && size <= 160) {
                if(printing_debug) std::cout<< " Dragon dead detected";
                // Dragon is dead
                if (dragon_types == "gdragon" && !gdragon){
                    gdragon = true;
                } else if (dragon_types == "ydragon" && !ydragon) {
                    ydragon = true;
                }
            } else if (size >= 165 && size < 180) {
                // Dragon is alive
                 if(printing_debug) std::cout<< " Dragon alive detected";
                if (dragon_types == "gdragon" && gdragon) {
                    gdragon = false;
                } else if (dragon_types == "ydragon" && ydragon) {
                    ydragon= false;
                }
            }
        }
    }
    
    // Dragon state functions
    bool ydragon_killed(const std::vector<pixel_t>& screen_pixels) const {
        if(ydragon) return true; 
        detect_dragons(screen_pixels, "ydragon");
        return ydragon;
    }
    
    bool gdragon_killed(const std::vector<pixel_t>& screen_pixels) const {
         if(gdragon) return true;
        detect_dragons(screen_pixels, "gdragon");
        return gdragon;
    }
    
    // Item distance function (MANHATTAN)
    int get_item_distance(const std::string& item_type, 
                          const std::vector<pixel_t>& screen_pixels) const 
    {
        const int MAX_DISTANCE = -1;
        auto cube_coords = find_cube_without_reference(screen_pixels);
        if (cube_coords.first == -1) return MAX_DISTANCE;
        std::pair<int,int> temp = get_cube_center(screen_pixels);    
        // Check entire screen
        auto entire_items = detect_items_entire_screen(screen_pixels);
        int min_dist = MAX_DISTANCE;
        for (const auto& [type, coord] : entire_items) {
            if (type == item_type) {
                int dist = manhattan_dist(coord.first , coord.second, temp.first , temp.second);
                if (dist < min_dist || min_dist== -1) min_dist = dist;
            }
        }
        if(min_dist == MAX_DISTANCE) {
            // Check around cube
            auto items_around_cube = detect_items_around_cube(screen_pixels, cube_coords);
            for (const auto& [type, coord] : items_around_cube) {
                if (type == item_type) {
                    int dist = manhattan_dist(coord.first , coord.second, temp.first , temp.second);
                    if (dist < min_dist || min_dist== -1) min_dist = dist;
                }
            }
        }
        if(min_dist == MAX_DISTANCE) {
            // Check around cube
            auto clusters = form_clusters_around_the_cube(screen_pixels, cube_coords);
            auto regions = regions_for_cube(screen_pixels);
             // Analyze filtered clusters
            for (const auto& cluster : clusters) {
                if (cluster.empty()) continue;
                
                // Check if cluster is in valid regions
                if (!cluster_in_regions(cluster, regions)) continue;

                // Calculate cluster centroid
                int sum_x = 0, sum_y = 0;
                for (const auto& [x, y] : cluster) {
                    sum_x += x;
                    sum_y += y;
                }
                int center_x = sum_x / cluster.size();
                int center_y = sum_y / cluster.size();
                
                // Calculate cluster size and color
                size_t size = cluster.size();
                auto first_pixel = *cluster.begin();
                pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];

                
                if (size >= 26 && size <= 30 && color_match(color, COLORS.at("yellow")) && item_type == "yellow_key") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                }else if (size >= 20 && size <= 25 && color_match(color, COLORS.at("yellow")) && item_type == "yellow_sword") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                }
                else if (size >= 26 && size <= 30 && color_match(color, COLORS.at("black")) && item_type == "black_key") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                } else if (size >= 67 && size <= 68 && item_type == "chalice") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                }
            } 
        }
         return min_dist;
    
    }
        
    // Updated item detection functions (FIXED TYPO IN chalice_dist)
    int ykey_dist(const std::vector<pixel_t>& screen_pixels) const {
        int dist = get_item_distance("yellow_key", screen_pixels);
        return dist;
    }
    int ysword_dist(const std::vector<pixel_t>& screen_pixels) const {
        return get_item_distance( "yellow_sword", screen_pixels); 
    }
    int bkey_dist(const std::vector<pixel_t>& screen_pixels) const {
        return get_item_distance( "black_key", screen_pixels); 
    }
    int chalice_dist(const std::vector<pixel_t>& screen_pixels) const {
        return get_item_distance( "chalice", screen_pixels); 
    }
    
    // Distance to navigation points (MANHATTAN)
    int dist_to_nav1(const std::vector<pixel_t>& screen_pixels) const {
        auto center = get_cube_center(screen_pixels);
        if (center.first == -1) return -1;
        int dist = manhattan_dist(center.first,center.second,27,158);
        if (abs(center.second-158) == 0) reachednav1 = true;
        return dist;
    }

    int dist_to_nav2(const std::vector<pixel_t>& screen_pixels) const {
        auto center = get_cube_center(screen_pixels);
        if (center.first == -1) return -1;
         int dist = manhattan_dist(center.first,center.second,78,149);
        if ( abs(center.first-78) == 0) reachednav2 = true;
        return dist;
    }

    int dist_to_nav3(const std::vector<pixel_t>& screen_pixels) const {
        auto center = get_cube_center(screen_pixels);
        if (center.first == -1) return -1;
        int dist = manhattan_dist(center.first,center.second,78,122);
        if (dist <= 2) reachednav3 = true;
        return dist;
    }
    // OPTIMIZED ITEM DETECTION WITH CLOSEST ITEM CHECK
    bool ykey(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);
        
        bool detected = detect_ykey_touching_cube(screen_pixels, cube_pos, "yellow_key", printing);
        if(printing) std::cout <<"detect_ykey_printing: " <<detected;
        if(printing_debug){
            if (detected ){  std::cout << "detected ykey touching and dist between cube and key is " << ykey_dist(screen_pixels)<< std::endl; 
                /* std::cout << "Screen: " << std::endl; 
                    for(int i = 0; i < screen_pixels.size(); i++){
                            std::cout << static_cast<int>( screen_pixels[i]) << " ";
                            if(i % SCREEN_WIDTH == 0 && i != 0) std::cout << std::endl;
                    }
                    std::cout<<std::endl << "Screen_ended" << std::endl; */

            }  
            else std::cout <<"detected no ykey and dist between cube and key is " << ykey_dist(screen_pixels)<< std::endl; 
        }
         if(printing_debug_adventure && detected != 0) std::cout << "detect_ykey_printing: " << detected << std::endl;
            return detected;
        
    }
    
    bool bkey(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image) const {
          auto cube_pos = highlight_cube(screen_pixels, prev_image);
          bool detected  =detect_ykey_touching_cube(screen_pixels, cube_pos, "black_key");
        if(printing_debug) std::cout <<"detect_bkey_printing: " <<detected; 
        return detected;
    }
    
    bool ysword(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);
        bool detected = detect_ykey_touching_cube(screen_pixels, cube_pos, "yellow_sword");
        //std::cout <<"detect_yswr_printing: " << detected; 
        return detect_ykey_touching_cube(screen_pixels, cube_pos, "yellow_sword");
    }
    
    bool chalice(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);
        bool detected = detect_ykey_touching_cube(screen_pixels, cube_pos, "chalice"); 
        if(printing_debug) std::cout <<"detect_chalice_printing: " << detected; 
        return detected;
    }

    bool ykeyr(const std::vector<pixel_t>& screen_pixels) const {
        auto items = detect_items_entire_screen(screen_pixels); 
        for(auto item : items){
            if(item.first == "yellow_key") return true; 
        }
        return false; 
    }
    bool bkeyr(const std::vector<pixel_t>& screen_pixels) const {
        auto items = detect_items_entire_screen(screen_pixels); 
        for(auto item : items){
            if(item.first == "black_key") return true; 
        }
        return false; 
    }
    bool chalicer(const std::vector<pixel_t>& screen_pixels) const {
        auto items = detect_items_entire_screen(screen_pixels); 
        for(auto item : items){
            if(item.first == "chalice") return true; 
        }
        return false; 
    }
    bool yswr(const std::vector<pixel_t>& screen_pixels) const {
        auto items = detect_items_entire_screen(screen_pixels); 
        for(auto item : items){
            if(item.first == "yellow_sword") return true; 
        }
        return false; 
    }
        
// Main detection function
    bool detect_ykey_touching_cube(const std::vector<pixel_t>& screen_pixels, const std::pair<int, int>& cube_pos, std::string type, bool printing = false) const {
        if (cube_pos.first == -1 || cube_pos.second == -1) {
            if(printing_debug) std::cout << "No cube found" << std::endl;
            return false; // No cube found
        }
        
        int cube_x = cube_pos.first;
        int cube_y = cube_pos.second;
        bool to_check = ykeyt; 
        if (type == "yellow_sword") to_check = yswrt;
        else if (type == "black_key") to_check = bkeyt;
        else if (type == "chalice") to_check = chalicet; 
        if( printing_debug|| printing ) std::cout<< std::endl << (to_check ?  "already touched" : "not touched ") << std::endl ; 
       
        if (to_check) {
            auto items = detect_items_around_cube(screen_pixels, cube_pos);
            for (const auto& item : items) {
                if (item.first == type) {
                    if (type == "yellow_sword") yswrt = true;
                    else if (type == "black_key")  bkeyt = true ;
                    else if (type == "chalice") chalicet = true; 
                    else ykeyt = true; 
                     if (printing_debug ||printing) std::cout<< "found cube carrying " << type << std::endl;
                    return true;
             
                }
            }
        }        
        bool found = false;
        // Don't have yellow key yet - check detected items first
        auto items = detect_items_entire_screen(screen_pixels);
        for (const auto& item : items) {
            if (item.first == type) {
                found = true;
                int key_x = item.second.first;
                int key_y = item.second.second;
                int cube_center_x = cube_x ; //+ adventure_cube_width / 2
                int cube_center_y = cube_y ; //+ adventure_cube_height / 2
                 if ( printing_debug||printing ) std::cout<< "found one " << type << " at" <<key_x << " " << key_y  << " cube at" << cube_x << " " << cube_y << " cubecenter at" << key_x+adventure_cube_width/2 << " " << key_y+adventure_cube_height/2 << std::endl;
                if(manhattan_dist(cube_center_x, cube_center_y, key_x, key_y) >= 5) { 
                 // Reset item state if far away
                reset_item_state();
                return false; 
                }
                
            }
        }
        if((printing_debug && !found)|| printing) std::cout<< " item not found in items" << std::endl;
        // If not found in items, search around cube
        // Define search area (expanded by 10 pixels)
       auto clusters = form_clusters_around_the_cube(screen_pixels, cube_pos);
        auto regions = regions_for_cube(screen_pixels);
        if (printing_debug || printing)std::cout << " Clusters found: " << clusters.size() << std::endl; 
        auto cube_boundary = get_cube_boundary(cube_pos);
        std::vector<std::set<std::pair<int, int>>> filtered_clusters;
        for (const auto& cluster : clusters) {
            // Count adjacent pixels to cube
            int adjacent_count = 0;
            for (const auto& pixel : cluster) {
                if (cube_boundary.find(pixel) != cube_boundary.end()) {
                    adjacent_count++;
                }
            }

            if (adjacent_count >= 2) filtered_clusters.push_back(cluster);
        }
        // Validate clusters

        for (const auto& cluster : filtered_clusters) {
            if (!cluster_in_regions(cluster, regions)) {
                continue;
            }
            
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
           
            // Check if it's a valid yellow key
            if (size >= 26 && size <= 30 && color_match(color, COLORS.at("yellow"))  && type == "yellow_key")  // Room color check
            {
                 if (printing_debug && count == 0){
                    for(int i= 0; i < screen_pixels.size(); i++){
                        std::cout << static_cast<int>(screen_pixels[i]) << " ";
                        if(i % SCREEN_WIDTH == 0 && i != 0) std::cout << std::endl;
                    }
                }
                 if ( printing_debug|| printing ) std::cout << " founnd ykeyt 1419" << std::endl;
                    count ++;

                ykeyt = true;
                bkeyt = false;
                yswrt = false;
                chalicet = false;
                return true;
            }else if (size >= 26 && size <= 30 && color_match(color, COLORS.at("black")) && type == "black_key") {
                bkeyt = true; 
                ykeyt = false;
                yswrt = false;
                chalicet = false;
                if (printing_debug) std::cout << " founnd bkeyt " << std::endl;
                return true; 
            }else if (size >= 20 && size <= 25 && color_match(color, COLORS.at("yellow")) && type == "yellow_sword") {
                yswrt = true; 
                ykeyt = false;
                bkeyt = false;
                chalicet = false;
                if (printing_debug) std::cout << " founnd yswrt " << std::endl;
                return true; 
            }else if (size >= 67 && size <= 68 && type == "chalice") {
                chalicet = true; 
                ykeyt = false;
                bkeyt = false;
                yswrt = false;
                if (printing_debug_adventure) std::cout << " founnd chalicet " << std::endl;
                return true; 
            }
        }
        if (printing_debug||printing) std::cout << " found nothing " << std::endl; 
        reset_item_state();
        return false;
    }
    std::vector<std::set<std::pair<int, int>>> form_clusters_around_the_cube(const std::vector<pixel_t>& screen_pixels, const std::pair<int, int>& cube_pos) const {
        auto cube_x = cube_pos.first;
        auto cube_y = cube_pos.second;
        int x_start = std::max(0, cube_x - 10);
        int y_start = std::max(0, cube_y - 10);
        int x_end = std::min(SCREEN_WIDTH, cube_x + adventure_cube_width + 10);
        int y_end = std::min(SCREEN_HEIGHT, cube_y + adventure_cube_height + 10);
        
        // Collect non-grey pixels in search area
        std::set<std::pair<int, int>> touching_pixels;
        for (int y = y_start; y < y_end; y++) {
            for (int x = x_start; x < x_end; x++) {
                // Skip cube area
                if (cube_x <= x && x < cube_x + adventure_cube_width &&
                    cube_y <= y && y < cube_y + adventure_cube_height) 
                {
                    continue;
                }
                    
                pixel_t pixel = screen_pixels[y * SCREEN_WIDTH + x];
                if (!is_grey(pixel)) {
                    touching_pixels.insert({x, y});
                }
            }
        }
        
        return cluster_pixels_using_seed(touching_pixels, screen_pixels);
    }
    
    void reset_item_state() const{
        ykeyt = false;
        bkeyt = false;
        yswrt = false;
        chalicet = false;
    }
    // Helper function to get cube boundary
    std::set<std::pair<int, int>> get_cube_boundary(const std::pair<int, int>& cube_pos) const {
        std::set<std::pair<int, int>> boundary;
        int x0 = cube_pos.first;
        int y0 = cube_pos.second;

        // Left boundary
        for (int y = y0; y < y0 + adventure_cube_height; y++) {
            if (x0 - 1 >= 0) boundary.insert({x0 - 1, y});
        }
        // Right boundary
        for (int y = y0; y < y0 + adventure_cube_height; y++) {
            if (x0 + adventure_cube_width < SCREEN_WIDTH) 
                boundary.insert({x0 + adventure_cube_width, y});
        }
        // Top boundary
        for (int x = x0; x < x0 + adventure_cube_width; x++) {
            if (y0 - 1 >= 0) boundary.insert({x, y0 - 1});
        }
        // Bottom boundary
        for (int x = x0; x < x0 + adventure_cube_width; x++) {
            if (y0 + adventure_cube_height < SCREEN_HEIGHT) 
                boundary.insert({x, y0 + adventure_cube_height});
        }

        return boundary;
    }
    std::vector<bool> check_sketches_preconditions (const std::vector<pixel_t>& pre, const std::vector<pixel_t>& post, const SimPlanner& planner, bool printing = false) const{
            std::vector<bool> sketches_pre(planner.sketches_.size(), false);
            for (size_t i = 0; i < sketches_.size(); ++i) {  
                    sketches_pre[i] = planner.sketches_[i].precondition(planner, pre, post);
            }
            /*// Only check if priority matches sketch index
                if(planner.priority_ > i) {
                    sketches_pre[i] = true;
                    
                }else if (i == planner.priority_) sketches_pre[i] = sketches_[i].precondition(planner, pre,post); 
                else sketches_pre[i] = false;*/
            if(printing) {
                std::cout << "Sketches pre: ";
                for (const auto& sketch : sketches_pre) {
                    std::cout << sketch << " ";
                }
                std::cout << std::endl;
            }
            return sketches_pre;
    }
    std::vector<bool> check_sketches_goals(const std::vector<pixel_t>& pre, const std::vector<pixel_t>& post, const std::vector<pixel_t>& prevs, const SimPlanner& planner, bool printing = false) const {
        std::vector<bool> sketches_post(sketches_.size(), false);
        // Check current priority sketch
        for(size_t i = 0; i < planner.sketches_.size(); ++i) {
            sketches_post[i] = planner.sketches_[i].goal(planner, pre, post, prevs);
        }
        if(printing) {
            std::cout << "Sketches post: ";
            bool key = planner.ykey(post,pre);
                int curr_dist = ykey_dist(post); 
                int prev_dist = ykey_dist(prevs);
                int D = planner.calculate_distance_from_goal(post);
                std::cout << "D: " << D << " | ykey: " << key << " | curr_dist: " << curr_dist << " | prevs_dist: " << prev_dist << std::endl;
                std::cout << "Sketches post: ";
                for (const auto& sketch : sketches_post) {
                    std::cout << sketch << " ";
                }
                std::cout << std::endl;
        }
        return sketches_post;
        /*
        if(planner.priority_ < sketches_.size()) {
            bool goal_achieved = sketches_[planner.priority_].goal(planner, pre, post, prevs);
            sketches_post[planner.priority_] = goal_achieved;
            
        }
            if(goal_achieved) {
                // Advance priority if goal achieved
                //std::cout << "goal achieved " << std::endl; 
                int D = planner.calculate_distance_from_goal(post); 
                int prior = planner.priority_; 
                bool local_ykey =  ykey(post,pre);
                if(printing_debug) std::cout<<std::endl << "Current priority: " << planner.priority_ << " D==" << D
                <<" ykey" << local_ykey << " reachednav1 " << reachednav1 << " reachednav2 " << reachednav2 << std::endl;
                // Check conditions to advance priority
                if (planner.priority_ == 0 && local_ykey  )   const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 1 &&  !local_ykey) {const_cast<SimPlanner&>(planner).priority_ = 0; reachednav1 = false; } 
                else if (planner.priority_ == 1 && local_ykey && reachednav1 )const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 2 && reachednav2 && local_ykey ) const_cast<SimPlanner&>(planner).priority_++;
                else if ((planner.priority_ == 3 || planner.priority_ == 15)&& D == 0 ) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 4 && ysword(post,pre) ) const_cast<SimPlanner&>(planner).priority_++;
                else if ((planner.priority_ == 5 || planner.priority_ == 12) && D==5 ) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 6 && ydragon) const_cast<SimPlanner&>(planner).priority_++;
                else if ((planner.priority_ == 7 || planner.priority_ == 11) && D==2 ) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 8 && D==4 ) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 9 && gdragon) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 10 && bkey(post,pre) ) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 13 && D == 9) const_cast<SimPlanner&>(planner).priority_++;
                else if (planner.priority_ == 14 && chalice(post,pre) ) const_cast<SimPlanner&>(planner).priority_++;
                if (prior != planner.priority_) {
                    if(impotant_debug) std::cout << "Advanced priority to: " << planner.priority_ << std::endl;
                    //std::cout<< "screeN::" << std::endl;
                    /*for(int i = 0; i < post.size(); i++){
                        std::cout << static_cast<int>( post[i]) << " ";
                        if(i % SCREEN_WIDTH == 0 && i != 0) std::cout << std::endl;
                    }
                    std::cout<<std::endl; 
                    //std::cout<< "screeN_ended" << std::endl;
                }
                // Mark all lower priorities as true (including the one we just completed)
                for(int i = 0; i < planner.priority_; i++) {
                    sketches_post[i] = true;
                }
            }*/
        
       
    }
    void initalize_sketches_adventure() {
        sketches_.clear();

        // Sketch 0: Acquire yellow key
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 0) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.ykey(curr,prev); 
                bool room = planner.ykeyr(curr);
                bool cond = D == 1 && (!key && room )  ; 
                if(printing_sketches_){
                std::cout<< std::endl; 
                std::cout << "SKETCH 0 PRE: D=" << D << " | !ykey=" << !key
                        << " | ykeyr=" << room << " | reachednav1=" << !reachednav1
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                bool key = planner.ykey(curr,prev);
                /*int curr_dist = ykey_dist(curr); 
                int prev_dist = ykey_dist(prev);
                if(prev_dist-curr_dist > 40 && impotant_debug && prev_dist >= 0 && curr_dist >= 0 ) std::cout << "ykey_dist: " << curr_dist << " (prev: " << prev_dist << ")" << std::endl;*/
                //int curr_dist = planner.ykey_dist(curr);
                //int prev_dist = ykey_dist(prevs);
                int D = planner.calculate_distance_from_goal(curr);
                bool acquiring_key = key ; // || (curr_dist+40 <= prev_dist && curr_dist >= 0 && prev_dist >= 0)
                bool goal_achieved =  acquiring_key && D==1;
                if(printing_sketches_){
                std::cout << "SKETCH 0 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | ykey=" << key 
                        //<< " | ykey_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                 }
                return goal_achieved;
            },
            "Acquire yellow key"
        });
        /*// Sketch 1: Navigate to black gate (nav2)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 1) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.ykey(curr,prev); 
                bool cond = D == 1 && key &&  !reachednav1 ;
                if(printing_sketches_){
                std::cout << "SKETCH 1 PRE: D=" << D << " | ykey=" << key 
                        << " | reachednav2=" << reachednav2 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int curr_dist = planner.dist_to_nav1(curr);
                int prev_dist = planner.dist_to_nav1(prev);
                int D = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (reachednav1 || (curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0)) && D ==1 ;
                if(printing_sketches_){
                std::cout << "SKETCH 1 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | nav2_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Reach nav 1"
        });

        // Sketch 1: Navigate to black gate (nav2)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 2) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.ykey(curr,prev); 
                bool cond = D == 1 && key && reachednav1 &&  !reachednav2 ;
                if(printing_sketches_){
                std::cout << "SKETCH 1 PRE: D=" << D << " | ykey=" << key 
                        << " | reachednav2=" << reachednav2 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int curr_dist = planner.dist_to_nav2(curr);
                int prev_dist = planner.dist_to_nav2(prev);
                int D = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (reachednav2 || (curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0)) && D ==1 ;
                if(printing_sketches_){
                std::cout << "SKETCH 1 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | nav2_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Reach black gate"
        });

        // Sketch 2: Navigate to inner gate (nav3)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 3) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.ykey(curr,prev); 
                bool cond = D == 1 && key  &&  reachednav2;
                if(printing_sketches_){
                std::cout << "SKETCH 2 PRE: D=" << D << " | ykey=" << key 
                        << " | reachednav2=" << reachednav2 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int curr_dist = planner.dist_to_nav3(curr);
                int prev_dist = planner.dist_to_nav3(prev);
                int D = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = ((curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0 )|| reachednav3) && D <=1;
                if(printing_sketches_){
                std::cout << "SKETCH 2 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | nav3_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Reach inner gate"
        });

        // Sketch 3: Acquire yellow sword
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 4) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool room = planner.yswr(curr);
                bool sword = planner.ysword(curr,prev);
                bool cond = D == 0 && !sword && room;
                if(printing_sketches_){
                std::cout << "SKETCH 3 PRE: D=" << D << " | !ysword=" << !sword 
                        << " | yswr=" << room 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                bool sword = planner.ysword(curr,prev);
                int curr_dist = ysword_dist(curr); 
                int prev_dist = ysword_dist(prev);
                int D = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (sword || (curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0)) && D == 0;
                if(printing_sketches_){
                std::cout << "SKETCH 3 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | ysword=" << sword 
                        << " | ysword_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Acquire yellow sword"
        });

        // Sketch 4: Navigate to yellow dragon room (D==5)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 5) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool sword = planner.ysword(curr,prev);
                bool cond = D >= 0 && D < 5 && sword;
                if(printing_sketches_){
                std::cout << "SKETCH 4 PRE: D=" << D << " | ysword=" << sword 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                int prevD = planner.calculate_distance_from_goal(prev);
                bool goal_achieved = (currD == 5) || currD > prevD;
                if(printing_sketches_){
                std::cout << "SKETCH 4 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << " (prev: " << prevD << ")" << std::endl;
                }
                return goal_achieved;
            },
            "Reach yellow dragon room"
        });

        // Sketch 5: Kill yellow dragon
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 6) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool dragon = !planner.ydragon_killed(curr);
                bool cond = D == 5 && dragon;
                if(printing_sketches_){
                std::cout << "SKETCH 5 PRE: D=" << D << " | !ydragon_killed=" << dragon 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int D = planner.calculate_distance_from_goal(curr);
                bool dragon = planner.ydragon_killed(curr);
                bool goal_achieved = dragon && D == 5;
                if(printing_sketches_){
                std::cout << "SKETCH 5 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | ydragon_killed=" << dragon 
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Kill yellow dragon"
        });

        // Sketch 6: Return to junction (D==2)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 7) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool cond = D == 5 && planner.ydragon_killed(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 6 PRE: D=" << D << " | ydragon_killed=" << planner.ydragon_killed(curr) 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                        return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (currD == 2);
                if(printing_sketches_){
                std::cout << "SKETCH 6 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << std::endl;
                }
                return goal_achieved;
            },
            "Return to junction"
        });

        // Sketch 7: Navigate to green dragon room (D==4)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 8) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool cond = D >= 2 && D < 4 && planner.ydragon_killed(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 7 PRE: D=" << D << " | ydragon_killed=" << planner.ydragon_killed(curr) 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                int prevD = planner.calculate_distance_from_goal(prev);
                bool goal_achieved = (currD == 4) || (currD > prevD);
                if(printing_sketches_){
                std::cout << "SKETCH 7 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << " (prev: " << prevD << ")" << std::endl;
                }
                return goal_achieved;
            },
            "Reach green dragon room"
        });

        // Sketch 8: Kill green dragon
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 9) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool cond = D == 4 && !planner.gdragon_killed(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 8 PRE: D=" << D << " | !gdragon_killed=" << !planner.gdragon_killed(curr) 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                bool goal_achieved = planner.gdragon_killed(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 8 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | gdragon_killed=" << goal_achieved << std::endl;
                }
                return goal_achieved;
            },
            "Kill green dragon"
        });

        // Sketch 9: Acquire black key
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 10) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.bkey(curr,prev);
                bool room = planner.bkeyr(curr);
                bool cond = D == 4 && planner.gdragon_killed(curr) && !key && room;
                if(printing_sketches_){
                std::cout << "SKETCH 9 PRE: D=" << D << " | gdragon_killed=" << planner.gdragon_killed(curr)
                        << " | !bkey=" << !key << " | bkeyr=" << room 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                bool key = planner.bkey(curr,prev);
                int D = planner.calculate_distance_from_goal(curr);
                int curr_dist = bkey_dist(curr); 
                int prev_dist = bkey_dist(prev);
                bool goal_achieved = (key || (curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0))  && D==4;
                if(printing_sketches_){
                std::cout << "SKETCH 9 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | bkey=" << key 
                        << " | bkey_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Acquire black key"
        });

        // Sketch 10: Return to junction with black key (D==2)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 11) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.bkey(curr,prev);
                bool cond = D <= 4 && D > 2 && key;
                if(printing_sketches_){
                std::cout << "SKETCH 10 PRE: D=" << D << " | bkey=" << key 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                int prevD = planner.calculate_distance_from_goal(prev);
                bool goal_achieved = (currD == 2) || currD < prevD;
                if(printing_sketches_){
                std::cout << "SKETCH 10 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << " (prev: " << prevD << ")" << std::endl;
                }   
                        return goal_achieved;
            },
            "Return to junction with black key"
        });

        // Sketch 11: Navigate to chalice path (D==5)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 12) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.bkey(curr,prev);
                bool cond = D == 2 && key;
                if(printing_sketches_){
                std::cout << "SKETCH 11 PRE: D=" << D << " | bkey=" << key 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                        return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (currD == 5);
                if(printing_sketches_){
                std::cout << "SKETCH 11 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << std::endl;
                }
                return goal_achieved;
            },
            "Reach chalice path"
        });

        // Sketch 12: Navigate to chalice room (D==9)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 13) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool cond = D >= 5 && D < 9 && bkeyt;
                if(printing_sketches_){
                std::cout << "SKETCH 12 PRE: D=" << D << " | bkeyt=" << bkeyt 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                        return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (currD == 9);

                if(printing_sketches_){
                std::cout << "SKETCH 12 GOAL: " << (goal_achieved ? "REACHED" : "MOVING")
                        << " | D_curr=" << currD << std::endl;
                }
                return goal_achieved;
            },
            "Reach chalice room"
        });

        // Sketch 13: Acquire chalice
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 14) return false;
                bool key = planner.chalice(curr,prev);
                bool room = planner.chalicer(curr);
                int D = planner.calculate_distance_from_goal(curr);
                bool cond = D == 9 && !key && room;
                if(printing_sketches_){
                std::cout << "SKETCH 13 PRE: D=" << D << " | !chalice=" << !key 
                        << " | chalicer=" << room 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                        return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                bool key = planner.chalice(curr,prev);
                int curr_dist = chalice_dist(curr);
                int prev_dist = chalice_dist(prev);
                int D = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (key || (curr_dist < prev_dist && curr_dist >= 0 && prev_dist >= 0 )) && D == 9;
                if(printing_sketches_){
                std::cout << "SKETCH 13 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | chalice=" << key 
                        << " | chalice_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        << " | D=" << D << std::endl;
                }
                return goal_achieved;
            },
            "Acquire chalice"
        });

        // Sketch 14: Return to start
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(planner.priority_ != 15) return false;
                int D = planner.calculate_distance_from_goal(curr);
                bool key = planner.chalice(curr,prev);
                bool cond = D <= 9 && key;
                if(printing_sketches_){
                std::cout << "SKETCH 14 PRE: D=" << D << " | chalice=" << key 
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                int currD = planner.calculate_distance_from_goal(curr);
                bool goal_achieved = (currD == 0);
                if(printing_sketches_){
                std::cout << "SKETCH 14 GOAL: " << (goal_achieved ? "VICTORY!" : "RETURNING")
                        << " | D_curr=" << currD << std::endl;
                }
                return goal_achieved;
            },
            "Return to start"
        });*/
    }
    
};
#endif

