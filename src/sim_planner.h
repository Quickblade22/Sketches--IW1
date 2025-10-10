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
struct RoomData {
    std::string name;
    std::pair<int, int> cube_position;
    std::vector<pixel_t> screen_pixels;
};
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
            //action_set_[0] = action_set_[5];
            action_set_.resize(6);
            for (size_t i = 0; i < action_set_.size(); ++i) {
                std::cout << "Action " << i << ": " << action_set_[i] << std::endl;
            }
        }
        reset_game(sim_);
        get_state(sim_, initial_sim_state_);
        load_database("Database.txt");
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
    const bool printing_sketches_functions = false; 
    mutable bool printing_sketches_debug = false; 
    mutable Node* current_node = nullptr; // Pointer to the current node being processed
    mutable int root_dist_to_sword = 0; 
    mutable int root_dist_to_key = 0;
    mutable int root_dist_to_chalice = 0; 
    mutable int root_dist_to_bkey  = 0; 
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
    mutable std::map<int, RoomData> room_database_;     
    const int adventure_cube_width = 4; // Width of the adventure cube in pixels
    const int adventure_cube_height = 8; // Height of the adventure cube in pixels
    mutable int count =0 ; 
    const pixel_t COLOR_THRESHOLD = 5;
    // Define the pattern as a 2D vector of pixel_t values
    const std::vector<std::vector<pixel_t>> pattern_dead = {
            {193, 170},
            {193, 193, 193, 193, 193, 193, 170, 170, 170},
            {193, 193, 193, 193, 193, 193, 170, 170, 170},
            {193, 193, 193, 193, 193, 193, 193, 170, 170, 170},
            {193, 193, 193, 193, 193, 193, 193, 170, 170, 170},
            {193, 193, 193, 193, 193, 193, 193, 170, 170, 170},
            {193, 193, 193, 193, 193, 193, 193, 170, 170, 170},
            {170, 193, 193, 193, 193, 193, 193, 170, 170, 170},
            {170, 193, 193, 193, 193, 193, 193, 170, 170},
            {170, 193, 193, 193, 193, 170, 170, 170},
            {170, 193, 193, 193, 193, 170, 170, 170},
            {170, 170, 193}
        };
        /* */
    const std::vector<std::vector<pixel_t>> pattern_alive = {
            {170, 170, 193, 193, 193, 193, 193, 193},
            {170, 170, 193, 193, 193, 193, 193, 193},
            {170, 193, 193, 193, 193, 193, 193, 193},
            {170, 193, 193, 193, 193, 193, 193, 193},
            {193, 193, 193, 170, 170, 170, 193, 193},
            {193, 193, 193, 170, 170, 170, 193, 193},
            {193, 193, 170, 170, 170, 170, 193, 193},
            {193, 193, 170, 170, 170, 170, 193, 193}
        };
    const std::vector<std::vector<pixel_t>> pattern_firing = {
            {170, 170, 170, 193, 193, 193, 170},
            {170, 170, 170, 193, 193, 193, 170},
            {170, 193, 193, 193, 193, 193, 170,170},
            {170, 193, 193, 193, 193, 193, 170,170},
            {170, 193, 193, 193, 193, 193, 193,170},
            {170, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {193, 193, 193, 193, 193, 193, 193,170},
            {170, 193, 193, 193, 193, 193, 170},
            {170, 193, 193, 193, 193, 193, 170},
            {170, 170, 193, 193, 193, 170},
            {170, 170, 193, 193, 193, 170},
            {170, 170, 170, 193, 170, 170}

        };
        /* {170, 193, 193, 193, 193, 193, 170}, 
            {170, 193, 193, 193, 193, 193, 170},
            {170, 170, 193, 193, 193, 170, 170},
            {170, 170, 193, 193, 193, 170, 170}*/
        
    
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
    bool item_surrounding_grey(const std::vector<pixel_t>& screen_pixels, size_t x, size_t y) const {
       bool left = (x-5 >= 0) && is_grey(screen_pixels[y * SCREEN_WIDTH + (x - 5)]);
       bool right = x+ 5 < SCREEN_WIDTH && is_grey(screen_pixels[y * SCREEN_WIDTH + (x + 5)]);
       bool up =( y-5 >= 0) && is_grey(screen_pixels[(y - 5) * SCREEN_WIDTH + x]);
       bool down = y+ 5 < SCREEN_HEIGHT && is_grey(screen_pixels[(y + 5) * SCREEN_WIDTH + x]);
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
    void calculate_distance_from_goal(const std::vector<pixel_t>& screen_pixels) const {
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
       // return Last_room_color;
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

        if(is_black){
            // Black throne room
                Last_room_color = 11; 
                regions.push_back({{7, 18}, {40, 178}});
                regions.push_back({{119, 18}, {151, 178}});
                regions.push_back({{7, 82}, {47, 178}});
                regions.push_back({{111, 82}, {151, 178}});
                regions.push_back({{7, 146}, {151, 178}});
                regions.push_back({{63, 146}, {96, 195}});
                regions.push_back({{71, 114}, {88, 154}}); //door
        } else if (is_yellow && color_match_at(80, 80, "yellow")) {
                // Yellow throne room
                Last_room_color = 1; // Set Last_room_color to 0 for yellow throne room
                regions.push_back({{7, 18}, {40, 178}});
                regions.push_back({{119, 18}, {152, 178}});
                regions.push_back({{7, 82}, {48, 146}});
                regions.push_back({{111, 82}, {151, 146}});
                regions.push_back({{7, 147}, {152, 178}});
                regions.push_back({{71, 114}, {88, 154}}); //door
                //std::cout<<"yellow_throne_room"<<std::endl; 
        } else if(is_yellow) {
                // Normal yellow/black room
                Last_room_color = 0; // Set Last_room_color to 1 for normal yellow room
                regions.push_back({{7, 18}, {152, 179}});
        }else if (is_red || is_pink) {
            if(is_pink) Last_room_color = 13; // Set Last_room_color to 9 for normal pink room
            else {
                if (Last_room_color == 3) {
                    Last_room_color = 4; // Special case for red in the yellow room
                } else if (Last_room_color == 11 || Last_room_color == 13) {
                    Last_room_color = 12; // Special case for red in the purple room
                
            }
            } // Set Last_room_color to 4 for normal red room
            regions.push_back({{7, 18}, {152, 179}});
        } 
        else if (is_green || is_light_green || is_purple) {

            if (is_light_green) {
                Last_room_color = 5; // Set Last_room_color to 4 for normal light green room
                regions.push_back({{12, 18}, {160, 178}});  // x >= 12
            } else if (is_purple) {
                Last_room_color = 3; // Set Last_room_color to 3 for normal purple room
                regions.push_back({{0, 18}, {147, 178}});   // x <= 147
            }else{
                Last_room_color = 2; // Set Last_room_color to 2 for normal green room
                regions.push_back({{0, 18}, {160, 178}});
            }
        } 
        
        else if (is_blue) {
            // Determine blue room type
            if (color_match_at(79, 6, "blue")) {
                // Blue room 1
                if (color_match_at(79, 194, "blue")) {
                    Last_room_color = 10; 
                    regions.push_back({{0, 18}, {160, 50}});       // Top-left
                    regions.push_back({{38, 18}, {48, 114}});      // Top-right
                    regions.push_back({{111, 18}, {120, 114}});    // Upper-left
                    regions.push_back({{15, 82}, {72, 115}});      // Upper-right
                    regions.push_back({{87, 82}, {144, 115}});     // Upper-right
                    regions.push_back({{15, 82}, {24, 179}});      // Upper-right
                    regions.push_back({{135, 82}, {144, 179}});    // Upper-right
                    regions.push_back({{135, 146}, {160, 179}});   // Upper-right
                    regions.push_back({{0, 146}, {24, 179}});      // Upper-right
                    regions.push_back({{103, 146}, {128, 179}});   // Upper-right
                    regions.push_back({{31, 146}, {56, 179}});     // Upper-right
                    regions.push_back({{31, 146}, {40, 194}});     // Upper-right
                    regions.push_back({{47, 146}, {56, 194}});     // Upper-right
                    regions.push_back({{103, 146}, {112, 194}});   // Upper-right
                    regions.push_back({{119, 146}, {127, 194}});   // Upper-right
                    regions.push_back({{63, 82}, {72, 195}});      // Upper-right
                    regions.push_back({{87, 82}, {96, 195}});      // Upper-right
                } else {
                    Last_room_color = 6; 
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
                
            } 
            else if (color_match_at(77, 185, "blue")) {
                Last_room_color = 7; 
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
                Last_room_color = 8; 
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
                Last_room_color = 9; 
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

        
        //calculate_distance_from_goal(screen_pixels); // Update Last_room_color based on current screen pixels
        return regions;
    }
    void printing_screen(const std::vector<pixel_t>& screen_pixels) const {
        
        std::cout << "Screen Pixels: " << std::endl;
        for (size_t y = 0; y < SCREEN_HEIGHT; ++y) {
            for (size_t x = 0; x < SCREEN_WIDTH; ++x) {
                size_t idx = y * SCREEN_WIDTH + x;
                std::cout << static_cast<int>(screen_pixels[idx]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "End of Screen Pixels" << std::endl;
    }
    bool item_surroundin_grey(const std::vector<pixel_t>& screen_pixels, size_t x, size_t y) const {
            bool left = x-5 >= 0 && is_grey(screen_pixels[y * SCREEN_WIDTH + (x -5 )]);
            bool right = x+ 5 < SCREEN_WIDTH && is_grey(screen_pixels[y * SCREEN_WIDTH + (x + 5)]);
            bool up = y-5 >= 0 && is_grey(screen_pixels[(y - 5) * SCREEN_WIDTH + x]);
            bool down = y+ 5 < SCREEN_HEIGHT && is_grey(screen_pixels[(y + 5) * SCREEN_WIDTH + x]);
            return (left || right) && ( up || down); 
        }
        
    std::pair<int,int> find_cube_without_reference(const std::vector<pixel_t>& current,   std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions) const{
        
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
    void load_database(const std::string& filename) const {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Warning: Could not open database file: " << filename << std::endl;
            return;
        }

        std::string line;
        RoomData current_room;
        int line_count = 0;
        int expected_lines = SCREEN_HEIGHT;
        bool reading_pixels = false;
        int current_room_num = -1;

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            // Room header: [Room Name]
            if (line[0] == '[') {
                if (reading_pixels) {
                    if (current_room_num != -1) {
                        room_database_[current_room_num] = current_room;
                    }
                    reading_pixels = false;
                }
                current_room = RoomData();
                current_room.name = line.substr(1, line.find(']') - 1);
                current_room_num = -1;
                continue;
            }

            // Room number line: "room_number: 11"
            if (line.find("room_number:") != std::string::npos) {
                size_t colon_pos = line.find(':');
                std::string num_str = line.substr(colon_pos + 1);
                // Trim whitespace
                num_str.erase(0, num_str.find_first_not_of(" \t"));
                num_str.erase(num_str.find_last_not_of(" \t") + 1);
                current_room_num = std::stoi(num_str);
                continue;
            }

            // Cube position line: "cube_position: 79,170"
            if (line.find("cube_position:") != std::string::npos) {
                size_t colon_pos = line.find(':');
                std::string pos_str = line.substr(colon_pos + 1);
                // Trim whitespace
                pos_str.erase(0, pos_str.find_first_not_of(" \t"));
                pos_str.erase(pos_str.find_last_not_of(" \t") + 1);
                size_t comma_pos = pos_str.find(',');
                int x = std::stoi(pos_str.substr(0, comma_pos));
                int y = std::stoi(pos_str.substr(comma_pos + 1));
                current_room.cube_position = {x, y};
                current_room.screen_pixels.clear();
                reading_pixels = true;
                line_count = 0;
                continue;
            }

            // Pixel data
            if (reading_pixels && line_count < expected_lines) {
                std::istringstream iss(line);
                int pixel_val;
                for (int i = 0; i < SCREEN_WIDTH; ++i) {
                    if (iss >> pixel_val) {
                        current_room.screen_pixels.push_back(static_cast<pixel_t>(pixel_val));
                    }
                }
                line_count++;
                if (line_count == expected_lines && current_room_num != -1) {
                    room_database_[current_room_num] = current_room;
                    reading_pixels = false;
                }
            }
        }
        // Save last room if needed
        if (reading_pixels && current_room_num != -1 && line_count == expected_lines) {
            room_database_[current_room_num] = current_room;
        }
    }
    std::pair<int, int> find_cube_using_database_comparison(const std::vector<pixel_t>& scene_img, bool printing = false) const {
        auto trial = regions_for_cube(scene_img); 
        int room_num = Last_room_color; // Use Last_room_color as room number
        auto it = room_database_.find(room_num);
        if (it == room_database_.end()) {
            if(printing) std::cout<< "Room not found in database." << std::endl;
            return {-1, -1};  // Room not in database
        }

        const RoomData& room_data = it->second;
        const std::vector<pixel_t>& db_img = room_data.screen_pixels;
        const auto& db_cube_pos = room_data.cube_position;

        // Collect candidate pixels: non-grey in scene but grey in DB
        std::set<std::pair<int, int>> candidate_pixels;
        for (int y = 0; y < SCREEN_HEIGHT; ++y) {
            for (int x = 0; x < SCREEN_WIDTH; ++x) {
                size_t idx = y * SCREEN_WIDTH + x;
                if (is_grey(db_img[idx]) && !is_grey(scene_img[idx])) {
                    candidate_pixels.insert({x, y});
                }
            }
        }
        int temp = candidate_pixels.size();
        if(printing) std::cout << "Found candidates pixels: " << candidate_pixels.size() << std::endl;
        // Also check around DB cube position
        pixel_t cube_color_db = db_img[db_cube_pos.second * SCREEN_WIDTH + db_cube_pos.first];
        for (int dy = -1; dy <= adventure_cube_height; ++dy) {
            for (int dx = -1; dx <= adventure_cube_width; ++dx) {
                int x = db_cube_pos.first + dx;
                int y = db_cube_pos.second + dy;
                if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) continue;
                size_t idx = y * SCREEN_WIDTH + x;
                if (color_match(db_img[idx], cube_color_db) && !is_grey(scene_img[idx])) {
                    candidate_pixels.insert({x, y});
                }
            }
        }
          if(printing) std::cout << "Found more candidates pixels: " << candidate_pixels.size()-temp << std::endl;
        // Cluster candidate pixels
        auto clusters = cluster_pixels(candidate_pixels, scene_img);
        if(printing) std::cout << "Number of clusters found: " << clusters.size() << std::endl;
        std::vector<std::set<std::pair<int, int>>> cube_clusters;
        for (const auto& cluster : clusters) {
            size_t size = cluster.size();
            //changed 0.7 --> 0.5
            if (size >= adventure_cube_width * adventure_cube_height * 0.5 && 
                size <= adventure_cube_width * adventure_cube_height) {
                cube_clusters.push_back(cluster);
            }
        }
        if(printing) std::cout << "Number of cube clusters found: " << cube_clusters.size() << std::endl;
        // Select best cluster (closest to DB cube position)
        if (cube_clusters.empty()) return {-1, -1};
        if (cube_clusters.size() == 1) {
            const auto& cluster = cube_clusters[0];
            int min_x = SCREEN_WIDTH, min_y = SCREEN_HEIGHT;
            for (const auto& pt : cluster) {
                if (pt.first < min_x) min_x = pt.first;
                if (pt.second < min_y) min_y = pt.second;
            }
            return {min_x, min_y};
        } else {
            int best_index = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (size_t i = 0; i < cube_clusters.size(); ++i) {
                int sum_x = 0, sum_y = 0;
                for (const auto& pt : cube_clusters[i]) {
                    sum_x += pt.first;
                    sum_y += pt.second;
                }
                int center_x = sum_x / cube_clusters[i].size();
                int center_y = sum_y / cube_clusters[i].size();
                float dist = std::hypot(center_x - db_cube_pos.first, center_y - db_cube_pos.second);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_index = i;
                }
            }
            const auto& cluster = cube_clusters[best_index];
            int min_x = SCREEN_WIDTH, min_y = SCREEN_HEIGHT;
            for (const auto& pt : cluster) {
                if (pt.first < min_x) min_x = pt.first;
                if (pt.second < min_y) min_y = pt.second;
            }
            return {min_x, min_y};
        }
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
        std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions = regions_for_cube(current);
        if((Last_room_color >= 6 && Last_room_color <= 10) ||Last_room_color == 4) {
           //std::cout<< "using database" << std::endl;
            temp = find_cube_using_database_comparison(current);  //for the blue room use 
            if (temp.first != -1) temp = find_cube_without_reference(current, regions); 
        }else{
              temp = find_cube_without_reference(current, regions); 
             
               //if (temp.first != -1) temp = find_cube_using_database_comparison(current);  
        }
        return temp;
    }
        // Helper function to get cube center coordinates
    std::pair<int, int> get_cube_center(const std::vector<pixel_t>& screen_pixels) const {
        auto cube_coords = highlight_cube(screen_pixels, screen_pixels);
        if (cube_coords.first == -1) return {-1, -1};
        return {
            cube_coords.first + adventure_cube_width / 2,
            cube_coords.second + adventure_cube_height / 2
        };
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
       //changed enforce that cube only touches items
        for (int x = x0; x < x0 + adventure_cube_width; x++) {
            if (y0 + adventure_cube_height < SCREEN_HEIGHT) 
                boundary.insert({x, y0 + adventure_cube_height});
        }
        return boundary;
    }
    // Clustering functions
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
    std::vector<std::pair<std::string, std::pair<int, int>>> detect_items_entire_screen(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_pixels, bool printing = false) const {
        std::vector<std::pair<std::string, std::pair<int, int>>> detected_items;
        auto regions = regions_for_cube(screen_pixels);
        auto cube_coords = highlight_cube(screen_pixels, screen_pixels);

        // Collect candidate pixels (non-grey) in entire screen
        std::set<std::pair<int, int>> candidates;
            for (int y = 0; y < SCREEN_HEIGHT; y++) {
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                // Skip cube area if found
                //changed avoid cube area --> get cube area reasoning: when cube on top of sword --> only sword being detected 
                /*if (cube_coords.first != -1 && 
                    x >= cube_coords.first && x < cube_coords.first + adventure_cube_width &&
                    y >= cube_coords.second && y < cube_coords.second + adventure_cube_height) {
                    continue;
                }*/
                
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
            pixel_t prev_color = prev_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];

            // Identify item type
            std::string item_type;
           if (size >= 26 && size <= 30 && color_match(color, COLORS.at("yellow"))) {
                item_type = "yellow_key";
            }else if (size >= 20 && size <= 25 && color_match(color, COLORS.at("yellow"))) { //22 px
                item_type = "yellow_sword";
            }
            else if (size >= 26 && size <= 30 && color_match(color, COLORS.at("black"))) {
                item_type = "black_key";
                //temporarily adding yellow as a nogo to motivate the cube to move further away
            } else if (size >= 66 && size <= 68 && check_pattern_chalice(screen_pixels, cluster)) {
                if(printing ){
                    bool same = true; 
                    for(int i = 0; i < screen_pixels.size(); i++){
                        if(screen_pixels[i] != prev_pixels[i]){
                            std::cout << "different screen pixels" << std::endl;
                            same = false;
                            break; 
                        }
                    }
                    if(same) std::cout << "same screen pixels" << std::endl;    
                    std::cout << "Chalice candidate at (" << first_pixel.first << ", " << first_pixel.second << ") with size " << size << " and color " << static_cast<int>(color) << " prev color is " << static_cast<int>(prev_color) << std::endl;
                }
                item_type = "chalice";
            }

            if (!item_type.empty() && item_surrounding_grey(screen_pixels, first_pixel.first, first_pixel.second)) {
                detected_items.push_back({item_type, {first_pixel.first, first_pixel.second}});
            }
        } 
        /* std::cout<<std::endl<<"Detected items entire screen" << std::endl; 
        for (auto i : detected_items){
            std::cout << i.first << " at " << i.second.first << " " << i.second.second << std::endl; 
        }*/
        return detected_items;
    }
    
    // Detect items near cube position
    std::vector<std::pair<std::string, std::pair<int, int>>> detect_items_around_cube(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_pixels, const std::pair<int, int>& cube_pos) const {
        if (cube_pos.first == -1 || cube_pos.second == -1) {
            return {};
        }
        
        const int padding = 15;
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
            pixel_t prev_color = prev_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
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
            else if (size >= 66 && size <= 68 && check_pattern_chalice(screen_pixels, cluster)) {
                
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
    
    
    //chalice pattern matching
    bool check_pattern_chalice(const std::vector<pixel_t>& screen_pixels, const std::set<std::pair<int, int>>& cluster) const {
        // Chalice pattern definition
        const std::vector<std::vector<pixel_t>> pattern_chalice = {
            {116, 170, 170, 170, 170, 170, 170, 116},
            {116, 170, 170, 170, 170, 170, 170, 116},
            {116, 170, 170, 170, 170, 170, 170, 116},
            {116, 116, 170, 170, 170, 170, 116, 116},
            {116, 116, 170, 170, 170, 170, 116, 116},
            {170, 116, 116, 116, 116, 116, 116, 170},
            {170, 116, 116, 116, 116, 116, 116, 170},
            {170, 116, 116, 116, 116, 116, 116, 170},
            {170, 116, 116, 116, 116, 116, 116, 170},
            {170, 170, 116, 116, 116, 116, 170, 170},
            {170, 170, 116, 116, 116, 116, 170, 170},
            {170, 170, 170, 116, 116, 170, 170, 170},
            {170, 170, 170, 116, 116, 170, 170, 170},
            {170, 170, 170, 116, 116, 170, 170, 170},
            {170, 170, 170, 116, 116, 170, 170, 170},
            {170, 116, 116, 116, 116, 116, 116, 170},
            {170, 116, 116, 116, 116, 116, 116, 170}
        };

        // First, find the chalice color (the non-grey color in the pattern)
        pixel_t chalice_color = 0;
        bool found_color = false;
        
        for (const auto& pixel : cluster) {
            int x = pixel.first;
            int y = pixel.second;
            size_t idx = static_cast<size_t>(y) * SCREEN_WIDTH + static_cast<size_t>(x);
            pixel_t pixel_val = screen_pixels[idx];
            
            if (!is_grey(pixel_val) && !color_match(pixel_val, COLORS.at("grey"))) {
                chalice_color = pixel_val;
                found_color = true;
                break;
            }
        }

        if (!found_color) {
            if (printing_debug) std::cout << "No chalice color found in cluster" << std::endl;
            return false;
        }

        // Try to match the pattern at different positions in the cluster
        for (const auto& start_pixel : cluster) {
            int start_x = start_pixel.first;
            int start_y = start_pixel.second;
            bool match = true;

            // Check if we have enough space for the pattern
            if (start_x + static_cast<int>(pattern_chalice[0].size()) > SCREEN_WIDTH ||
                start_y + static_cast<int>(pattern_chalice.size()) > SCREEN_HEIGHT) {
                continue;
            }

            // Check each pixel in the pattern
            for (size_t py = 0; py < pattern_chalice.size() && match; py++) {
                for (size_t px = 0; px < pattern_chalice[py].size() && match; px++) {
                    int scene_x = start_x + static_cast<int>(px);
                    int scene_y = start_y + static_cast<int>(py);
                    size_t scene_idx = static_cast<size_t>(scene_y) * SCREEN_WIDTH + static_cast<size_t>(scene_x);
                    pixel_t scene_pixel = screen_pixels[scene_idx];
                    pixel_t pattern_val = pattern_chalice[py][px];

                    // Pattern value 116 should match chalice color, 170 should be grey
                    if (pattern_val == 116) {
                        if (!color_match(scene_pixel, chalice_color)) {
                            match = false;
                            break;
                        }
                    } else { // pattern_val == 170
                        if (!(is_grey(scene_pixel) || color_match(scene_pixel, COLORS.at("grey")))) {
                            match = false;
                            break;
                        }
                    }
                }
            }

            if (match) {
                if (printing_debug) {
                    std::cout << "Chalice pattern matched at (" << start_x << ", " << start_y << ")" << std::endl;
                }
                return true;
            }
        }

        if (printing_debug) {
            std::cout << "Chalice pattern not found in cluster" << std::endl;
        }
        return false;
    }

    std::vector<std::set<std::pair<int, int>>> dragon_helper_function(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_pixels, bool printing = false) const {
       auto regions = regions_for_cube(screen_pixels);
        //update needed
        auto cube_coords = highlight_cube(screen_pixels,screen_pixels);
        
        // Get all items to exclude (entire screen)
        auto entire_items = detect_items_entire_screen(screen_pixels,prev_pixels);

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
        if(printing_debug||printing) std::cout<<"cluster size" << clusters.size(); 
       
        // NEW: Filter clusters  using item centroids
        std::set<std::pair<int, int>> exclusion_centroids;
        for (const auto& item : all_items) {
            exclusion_centroids.insert(item.second);
        }
        auto filtered_clusters = clusters; 
        if(printing_debug) std::cout<< "the size of exlcusion points" << exclusion_centroids.size();
        if(!exclusion_centroids.empty())  filtered_clusters = filter_clusters_by_exclusion_points(clusters, exclusion_centroids);
        //changed adding size exclusion to avoid small noise
        /*for (auto it = filtered_clusters.begin(); it != filtered_clusters.end(); ) {
            if (it->size() < 100 || it->size() >= 180) { // Minimum size threshold
                it = filtered_clusters.erase(it);
            } else {
                ++it;
            }
        }*/
        if(printing){
            for(auto c: clusters) {
                auto first_pixel = *c.begin();
                pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
                bool is_excluded = std::find(filtered_clusters.begin(), filtered_clusters.end(), c) != filtered_clusters.end();
                std::cout << "Cluster size: " << c.size() << " color: " << static_cast<int>(color) << " in filtered: " << is_excluded << std::endl;
                std::cout << std::endl;
            }
        }
        return filtered_clusters;
    }
    // Dragon detection function
    void detect_dragons(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels, std::string dragon_type, bool printing = false) const {
        auto filtered_clusters = dragon_helper_function(screen_pixels, previous_pixels, printing);
        if(printing_debug|| printing) std::cout<<"cluster size after filtering" << filtered_clusters.size() << std::endl; 
        for (const auto& cluster : filtered_clusters) {
            if(printing) std::cout << "filtered cluster with size: " << cluster.size() << std::endl;
            //changed 140 --> 100-->105
            if (cluster.size() < 100 || cluster.size() >= 180) continue; // Dragon size threshold
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            
            std::string dragon_types;
            if (dragon_type == "gdragon" && color_match(color, COLORS.at("gdragon"))) {
                dragon_types = "gdragon";
                if(printing) std::cout << "Detected " << dragon_types << " with color: " << static_cast<int>(color) << std::endl;
            } else if (dragon_type == "ydragon" && color_match(color, COLORS.at("ydragon"))) {
                dragon_types = "ydragon";
                if(printing) std::cout << "Detected " << dragon_types << " with color: " << static_cast<int>(color) << std::endl;
            } else {
                continue;
            }
            
            // Determine state by size
            size_t size = cluster.size();
            //tells us if dragon dead using pattern matching
            bool is_dead = check_dragon_pattern(screen_pixels, cluster, dragon_types);
            bool is_alive = check_dragon_pattern_alive(screen_pixels, cluster, dragon_types);
            bool is_firing = check_dragon_pattern_firing(screen_pixels, cluster, dragon_types, printing);
            if(printing) std:: cout << " is_dead" << is_dead <<  " is_alive" << is_alive <<  " is_firing" << is_firing << std::endl;
             if(is_dead){
                if(printing_debug||printing) std::cout<< " Dragon dead detected by pattern" << std::endl;
                if(dragon_types == "gdragon" && !gdragon){
                    gdragon = true;
                } else if(dragon_types == "ydragon" && !ydragon){
                    ydragon = true;
                }
                break;
            }else if(is_alive ||is_firing){
                if(printing_debug||printing) std::cout<< " Dragon alive detected by pattern" << std::endl;
                if(dragon_types == "gdragon" && gdragon){
                    gdragon = false;
                } else if(dragon_types == "ydragon" && ydragon){
                    ydragon = false;
                }
                break;
            } else {
                //changed: 140-->130 
                if (size >= 130 && size <= 142) {
                    if(printing_debug||printing) std::cout<< " Dragon dead detected by size" << std::endl;
                    // Dragon is dead
                    if (dragon_types == "gdragon" && !gdragon) gdragon = true;   
                    else if (dragon_types == "ydragon" && !ydragon) ydragon = true; //std::cout<< " ydragon is dead, cause found cluster" << cluster.size() << " with color" << static_cast<int>(color) << " at node: " << current_node->action_ << std::endl;
                    break;
                } else if (size > 142 && size < 180) {
                    // Dragon is alive
                    if(printing_debug||printing) std::cout<< " Dragon alive detected by size" << std::endl;
                    if (dragon_types == "gdragon" && gdragon) gdragon = false;
                    else if (dragon_types == "ydragon" && ydragon) ydragon= false; 
                    break;
                }
            }
        }
    }
    
    
    bool detect_dragons_in_room(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& previous_pixels, std::string dragon_type, bool printing = false) const {
       auto filtered_clusters = dragon_helper_function(screen_pixels, previous_pixels);
       //got the filtered clusters
        for (const auto& cluster : filtered_clusters) {
            //changed 140 --> 100-->105
            if (cluster.size() < 100 || cluster.size() >= 180) continue; // Dragon size threshold
            
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            size_t size = cluster.size();
            //check if dragon alive in room
            bool alive = check_dragon_pattern_alive(screen_pixels, cluster, dragon_type);
            bool firing = check_dragon_pattern_firing(screen_pixels, cluster, dragon_type, printing);
            bool dead  = check_dragon_pattern(screen_pixels, cluster, dragon_type);
            bool coloring_true = (dragon_type == "gdragon" && color_match(color, COLORS.at("gdragon"))) || (dragon_type == "ydragon" && color_match(color, COLORS.at("ydragon")));
            if(printing) std:: cout << "dragon_type " << dragon_type << "detect_dragons_in_room:  Cluster size " << cluster.size() << " is_alive " << alive <<  " is_firing " << firing << " is_dead " << dead << std::endl;
            if(alive || firing || dead) return coloring_true;
            //changed: only alive dragon to all dragon dead, firing, alive --> changed 165 to 142 and addded second or clause --> 142 turned into 140
            //chnaged: size detectes all dragon sizes
            //changed : 140 -->130-->101
            if(dragon_type == "gdragon" && printing && coloring_true) std::cout<< "cluster size: " << size << " color: " << static_cast<int>(color) << std::endl;
            else if((size >= 130 && size < 180) ) { //
                //check if dragon type matches
                if((printing_debug||printing)) std::cout<< " Dragon detected by size" << std::endl;
                return coloring_true;
            }
        }
        return false; // No dragon detected
    }
    // Dragon state functions
    // Add this helper function to check for dragon patterns
    bool check_dragon_pattern(const std::vector<pixel_t>& screen_pixels, const std::set<std::pair<int, int>>& cluster, const std::string& dragon_type) const {
        // Pattern for alive yellow dragon
        auto pattern = pattern_dead;
        // Check pattern at various positions in the cluster
        for (const auto& pixel : cluster) {
            int x = pixel.first;
            int y = pixel.second;
            bool match = true;
            for (int i = 0; i < pattern.size() && match; i++) {
                for (int j = 0; j < pattern[i].size() && match; j++) {
                    int check_x = x + j; // Center the pattern
                    int check_y = y + i;
                    
                    if (check_x < 0 || check_x >= SCREEN_WIDTH || 
                        check_y < 0 || check_y >= SCREEN_HEIGHT) {
                        match = false;
                        break;
                    }
                    
                    size_t idx = check_y * SCREEN_WIDTH + check_x;
                    ale::pixel_t color = dragon_type == "ydragon" ? pattern[i][j] : COLORS.at("gdragon");
                    if (!color_match(screen_pixels[idx], color)) {
                        match = false;
                        break; 
                    }
                }
                if(!match) break;    
            }
            if(match) {
                return true;
            }
        }
        
        return false;
    }
     // Add this helper function to check for dragon patterns
    bool check_dragon_pattern_alive(const std::vector<pixel_t>& screen_pixels, const std::set<std::pair<int, int>>& cluster, const std::string& dragon_type) const {
        // Pattern for alive yellow dragon
        auto pattern = pattern_alive;
        // Check pattern at various positions in the cluster
        for (const auto& pixel : cluster) {
            int x = pixel.first;
            int y = pixel.second;
            bool match = true;
            for (int i = 0; i < pattern.size() && match; i++) {
                for (int j = 0; j < pattern[i].size() && match; j++) {
                    int check_x = x + j-2; // Center the pattern
                    int check_y = y + i;
                    
                    if (check_x < 0 || check_x >= SCREEN_WIDTH || 
                        check_y < 0 || check_y >= SCREEN_HEIGHT) {
                        match = false;
                        break;
                    }
                    
                    size_t idx = check_y * SCREEN_WIDTH + check_x;
                    ale::pixel_t color = dragon_type == "ydragon" ? pattern[i][j] : COLORS.at("gdragon");
                    if (!color_match(screen_pixels[idx], color)) {
                        match = false;
                        break; 
                    }
                }
                if(!match) break; 
                
            }
            if(match) return true;
        }
        
        return false;
    }
 // Add this helper function to check for dragon patterns
    bool check_dragon_pattern_firing(const std::vector<pixel_t>& screen_pixels, const std::set<std::pair<int, int>>& cluster, const std::string& dragon_type, bool printing = false) const {
        // Pattern for alive yellow dragon
        //changed  pattern firing to pattern alive --> need to update pattern firing
        auto pattern = pattern_firing;
        // Check pattern at various positions in the cluster
        for (const auto& pixel : cluster) {
            int x = pixel.first;
            int y = pixel.second;
            bool match = true;
            for (int i = 0; i < pattern.size() && match; i++) {
                for (int j = 0; j < pattern[i].size() && match; j++) {
                    int check_x = x + j-3; // Center the pattern
                    int check_y = y + i;
                    
                    if (check_x < 0 || check_x >= SCREEN_WIDTH || 
                        check_y < 0 || check_y >= SCREEN_HEIGHT) {
                        match = false;
                        break;
                    }
                    
                    size_t idx = check_y * SCREEN_WIDTH + check_x;
                    ale::pixel_t color = dragon_type == "ydragon" ? pattern[i][j] : COLORS.at("gdragon");
                    if (!color_match(screen_pixels[idx], color)) {
                        match = false;
                        break; 
                    }
                }
                if(!match) break; 
                
            }
            //if(match)  printing_screen(screen_pixels);
            if(match) return true;
            if(match && printing){
                std::cout << "Firing dragon detected at " << x << " " << y << std::endl;
                printing_screen(screen_pixels);
            } 
        }
        
        return false;
    }

    
    bool ydragon_killed(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& previous_pixels, bool printing = false) const {
        if(printing) std::cout << "ydragon function called" << std::endl; 
        if(ydragon) return true; 
        if(printing) std::cout << "ydragon detecte dragons called" << std::endl; 
        detect_dragons(screen_pixels, previous_pixels, "ydragon", printing);
        return ydragon;
    }

    bool gdragon_killed(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels, bool printing = false) const {
        if(printing) std::cout << "gdragon function called" << std::endl; 
        if(gdragon) return true;
        detect_dragons(screen_pixels, previous_pixels, "gdragon", printing);
        return gdragon;
    }
    // Item distance function (MANHATTAN)
    int get_item_distance(const std::string& item_type, const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        const int MAX_DISTANCE = -1;
        auto cube_coords = highlight_cube(screen_pixels, screen_pixels);
        if (cube_coords.first == -1) return MAX_DISTANCE;
        std::pair<int,int> temp = get_cube_center(screen_pixels);    
        // Check entire screen
        auto entire_items = detect_items_entire_screen(screen_pixels,previous_pixels );
        int min_dist = MAX_DISTANCE;
        for (const auto& [type, coord] : entire_items) {
            if (type == item_type) {
                int dist = manhattan_dist(coord.first , coord.second, temp.first , temp.second);
                if (dist < min_dist || min_dist== -1) min_dist = dist;
            }
        }
        if(min_dist == MAX_DISTANCE) {
            // Check around cube
            auto items_around_cube = detect_items_around_cube(screen_pixels, previous_pixels, cube_coords);
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
                    //changed 25 --> 22
                }else if (size >= 20 && size <= 22 && color_match(color, COLORS.at("yellow")) && item_type == "yellow_sword") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                }
                else if (size >= 26 && size <= 30 && color_match(color, COLORS.at("black")) && item_type == "black_key") {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                    //temporarily adding yellow color as a no to motivate the cube to move further 
                } else if (size >= 66 && size <= 68 && item_type == "chalice" && check_pattern_chalice(screen_pixels, cluster)) {
                    return manhattan_dist(center_x, center_y, temp.first, temp.second);
                }
            } 
        }
         return min_dist;
    
    }
        
    // Updated item detection functions (FIXED TYPO IN chalice_dist)
    int ykey_dist(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        int dist = get_item_distance("yellow_key", screen_pixels, previous_pixels);
        return dist;
    }
    int ysword_dist(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        return get_item_distance( "yellow_sword", screen_pixels, previous_pixels); 
    }
    int bkey_dist(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        return get_item_distance( "black_key", screen_pixels, previous_pixels); 
    }
    int chalice_dist(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        return get_item_distance( "chalice", screen_pixels, previous_pixels); 
    }
    //changing size 142 --> 100-->105 && now adding pattern matching 
    int sword_dist_to_ydragon(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& previous_pixels) const {
        //get sword coordinates
       auto items = detect_items_entire_screen(screen_pixels, previous_pixels);
       std::pair<int, int> sword = {-1, -1};
       for(auto item: items) {
           if(item.first == "yellow_sword") {
               sword = item.second; 
               break; 
           }
       }

       if(sword.first == -1 || sword.second == -1) return -1;
       //get ydragon coord
       std::pair<int, int> ydragon_location = {-1, -1};
       auto dragons = dragon_helper_function(screen_pixels, previous_pixels);
       bool found = false; 
       for (const auto& cluster : dragons) {
           ydragon_location = *cluster.begin();
           pixel_t color = screen_pixels[ydragon_location.second * SCREEN_WIDTH + ydragon_location.first];
           // Determine state by size
           size_t size = cluster.size();
           if (size >= 100 && size < 180 && color == COLORS.at("ydragon")) {
               // Dragon is alive
               found = true; 
               break;
           }
       }
       if(ydragon_location.first == -1 || ydragon_location.second == -1 || !found) return -2;
       return manhattan_dist(sword.first, sword.second, ydragon_location.first, ydragon_location.second);
       //return manhattan_dist(sword.first, sword.second, ydragon_location.first, ydragon_location.second);
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
        auto center = highlight_cube(screen_pixels, screen_pixels);
        if (center.first == -1) return -1;
        int dist = manhattan_dist(center.first,center.second,80,128);
        return dist;
    }

    //key stays near door for 3 consecutive frames
    /*
    bool stay_near_door(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prevs ) const {
         auto items = detect_items_entire_screen(screen_pixels,prevs);
         bool curr_screen = false; 
         int x = 0; 
         int y = 0;
        for (const auto& item : items) {
            if (item.first == "yellow_key") {
                x = item.second.first;
                y = item.second.second;
                // Check if in door area: x in [72,89] and y in [146,148]
                if (x >= 72 && x <= 89 && y >= 146 && y <= 148) {
                    curr_screen = true; 
                    break;
                }
            }
        }
        
        items = detect_items_entire_screen(prevs);
        bool prev_screen = false;
         for (const auto& item : items) {
            if (item.first == "yellow_key") {
                if(item.second.first == x && item.second.second == y) {
                    prev_screen = true; 
                    break;
                }
            }
        }
        
        return curr_screen && prev_screen;
    }
    */
    // OPTIMIZED ITEM DETECTION WITH CLOSEST ITEM CHECK
    bool ykey(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);

        bool detected = detect_ykey_touching_cube(screen_pixels, prev_image, cube_pos, "yellow_key", printing);
        if(printing) std::cout <<"detect_ykey_printing: " <<detected << std::endl;
        if(printing_debug){
            if (detected ){  std::cout << "detected ykey touching and dist between cube and key is " << ykey_dist(screen_pixels, prev_image)<< std::endl; 
                /* std::cout << "Screen: " << std::endl; 
                    for(int i = 0; i < screen_pixels.size(); i++){
                            std::cout << static_cast<int>( screen_pixels[i]) << " ";
                            if(i % SCREEN_WIDTH == 0 && i != 0) std::cout << std::endl;
                    }
                    std::cout<<std::endl << "Screen_ended" << std::endl; */

            }  
            else std::cout <<"detected no ykey and dist between cube and key is " << ykey_dist(screen_pixels, prev_image)<< std::endl; 
        }
            return detected;
        
    }
    bool bkey(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
          auto cube_pos = highlight_cube(screen_pixels, prev_image);
          bool detected  = detect_ykey_touching_cube(screen_pixels, prev_image, cube_pos, "black_key", printing);
        if(printing) std::cout <<"detect_bkey_printing: " <<detected << std::endl; 
        //if(detected && printing) printing_screen(screen_pixels);
        return detected;
    }
    
    bool ysword(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);
        bool detected = detect_ykey_touching_cube(screen_pixels, prev_image, cube_pos, "yellow_sword", printing);
        //std::cout <<"detect_yswr_printing: " << detected; 
        if(printing) std::cout <<"detect_ysword_printing: " <<detected << std::endl;
        return detected;
    }
    
    bool chalice(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
        auto cube_pos = highlight_cube(screen_pixels, prev_image);
        bool detected = detect_ykey_touching_cube(screen_pixels, prev_image, cube_pos, "chalice", printing);
        if(printing) std::cout <<"detect_chalice_printing: " << detected; 
        return detected;
    }
    // Check if item is in room
    bool ykeyr(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image) const {
        auto items = detect_items_entire_screen(screen_pixels, prev_image); 
        for(auto item : items){
            if(item.first == "yellow_key") return true; 
        }
        //picked up yellow sword, so not dettecting it
        return (current_node != nullptr  && current_node->parent_ != nullptr && current_node->parent_->node_ykeyt == false && ykeyt);
    }
    bool bkeyr(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image) const {
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        for (auto item : items) {
            if (item.first == "black_key") return true;
        }
        //picked up yellow sword, so not dettecting it
        return (current_node != nullptr && current_node->parent_ != nullptr && current_node->parent_->node_bkeyt == false && bkeyt);
    }
    bool chalicer(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image, bool printing = false) const {
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        for (auto item : items) {
            if (item.first == "chalice") {
                if(printing) std::cout << "chalicer: chalice detected in room at "<< item.second.first << " ," << item.second.second << " with color " << static_cast<int>(screen_pixels[item.second.second * SCREEN_WIDTH + item.second.first]) << " or " 
                << static_cast<int>(screen_pixels[item.second.first * SCREEN_WIDTH + item.second.second]) << std::endl;
                return true;
            }
        }
        
        //picked up yellow sword, so not dettecting it
        return (current_node != nullptr && current_node->parent_ != nullptr && current_node->parent_->node_chalicet == false && chalicet);
    }
    bool yswr(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image) const {
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        for (auto item : items) {
            if (item.first == "yellow_sword") return true;
        }
        //picked up yellow sword, so not dettecting it
        return (current_node != nullptr && current_node->parent_ != nullptr && current_node->parent_->node_yswrt == false && yswrt);
    }
    bool ydragonr(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image, bool printing = false) const {
        return detect_dragons_in_room(screen_pixels,prev_image, "ydragon", printing);
    }
    bool gdragonr(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image, bool printing = false) const {
        if(printing) std::cout << "gdragonr function called" << std::endl;
        return detect_dragons_in_room(screen_pixels,prev_image, "gdragon", printing);
    }

    bool door_open(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image) const {
     //calculate_distance_from_goal(screen_pixels);
     int D = Last_room_color;
     if(D != 1 || D != 7) return false;
     int count = 0; 
     for(int y = 126; y <= 146; ++y) {
        count += is_grey(screen_pixels[y*SCREEN_WIDTH + 80]) ? 1 : 0;
     }
     if(impotant_debug) std::cout << "door_open count: " << count << std::endl;
     return count >= 15; // Assuming door_open color is at this pixel 80 123
    }
    // Check if cube touching item or already touched --> carrying item
    bool detect_ykey_touching_cube(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image, const std::pair<int, int>& cube_poss, std::string type, bool printing = false) const {
        //changed added find_cube_using_database_comparison
        std::pair<int, int> cube_pos = cube_poss;
        if (cube_poss.first == -1 || cube_poss.second == -1) {
            if(printing) std::cout << "No cube found, using database" << std::endl;
            /* cube_pos = find_cube_using_database_comparison(screen_pixels, printing);
                                if(cube_pos.first == -1 || cube_pos.second == -1) { }*/
                if(printing) std::cout << "No cube found using database comparison" << std::endl;
                if(printing) printing_screen(screen_pixels);
                return false; // No cube found
        }
        
        int cube_x = cube_pos.first;
        int cube_y = cube_pos.second;
        bool to_check = ykeyt; 
        if (type == "yellow_sword") to_check = yswrt;
        else if (type == "black_key") to_check = bkeyt;
        else if (type == "chalice") to_check = chalicet; 
        if(printing ) std::cout<< std::endl << (to_check ?  "already touched" : "not touched ") << std::endl ; 
        if(to_check && current_node != nullptr && current_node->action_ == 1){
            //std::cout << "already touched item, but action is 1, so resetting state and parent is " << current_node->parent_->action_ << std::endl;
            reset_item_state();
            return false; 
        }
       
        if (to_check) {
            auto items = detect_items_around_cube(screen_pixels,prev_image, cube_pos);
            for (const auto& item : items) {
                if (item.first == type) {
                    if (type == "yellow_sword") yswrt = true;
                    else if (type == "black_key")  bkeyt = true ;
                    else if (type == "chalice") chalicet = true; 
                    else ykeyt = true; 
                    if (printing_debug ||printing) std::cout<< "found cube carrying " << type << std::endl;
                    if( type == "black_key" && printing) std::cout << "black key detected near cube at " << item.second.first << " " << item.second.second << " and cube at "<< cube_pos.first<< " ," << cube_pos.second << std::endl;
                    return true;
             
                }
            }
            //changed added return true
            return true;
        }        
        bool found = false;
        // Don't have yellow key yet - check detected items first
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        if(printing_debug || printing) {
            std::cout << "Checking items in entire screen for type: " << type << std::endl;
            for (const auto& item : items) {
            int key_x = item.second.first;
            int key_y = item.second.second;
            std::cout<< "found " << item.first << " at " << key_x << " " << key_y  << " cube at " 
            << cube_x << " " << cube_y << std::endl;
            }
        }
        auto cube_boundary = get_cube_boundary(cube_pos);
        // check if item is already in items and close to cube ?
        for (const auto& item : items) {
            int key_x = item.second.first;
            int key_y = item.second.second;
            if (item.first == type) {
                found = true;
                int cube_center_x = cube_x ;// + adventure_cube_width / 2;
                int cube_center_y = cube_y  ;//+ adventure_cube_height / 2;
                for (const auto& bound_pixel : cube_boundary) {
                    if (manhattan_dist(key_x, key_y, bound_pixel.first, bound_pixel.second) <= 1) {
                    // Adjacent item found - set state and return
                    if (type == "yellow_sword") yswrt = true;
                    else if (type == "black_key") bkeyt = true;
                    else if (type == "chalice") chalicet = true;
                    else ykeyt = true;
                    
                    if (printing_debug || printing) 
                        std::cout << "Found adjacent " << type << " at (" 
                                 << key_x << "," << key_y << ")\n";
                    return true;
                    } 
                }
                if(manhattan_dist(cube_center_x, cube_center_y, key_x, key_y) >= 5) { 
                 // Reset item state if far away
                if( to_check) reset_item_state();
                if(printing_debug || printing) std::cout << " Item is too far from cube" << cube_x << " " << cube_y << " center: " << cube_x+adventure_cube_width /2 << " " << cube_center_y+adventure_cube_height / 2 << " | item: " << key_x << " " << key_y << std::endl;
                return false; 
                }
                
            }
        }
        if((printing_debug && !found)|| printing) std::cout<< " searching around cube" << std::endl;
        // If not found in items, search around cube
        // Define search area (expanded by 10 pixels)
       auto clusters = form_clusters_around_the_cube(screen_pixels, cube_pos);
        auto regions = regions_for_cube(screen_pixels);
        if (printing_debug )std::cout << " Clusters found: " << clusters.size() << std::endl; 
      
        std::vector<std::set<std::pair<int, int>>> filtered_clusters;
        
        for (const auto& cluster : clusters) {
            // Count adjacent pixels to cube
            int adjacent_count = 0;
            for (const auto& pixel : cluster) {
                if (cube_boundary.find(pixel) != cube_boundary.end()) {
                    adjacent_count++;
                }
            }
            //changed  (sofar 2 now 3)
            if (adjacent_count >= 3) filtered_clusters.push_back(cluster);
        }
        
        if(printing){
            std::cout << "Cube position: (" << cube_x << ", " << cube_y << ")" << std::endl;
            std::cout << " Clusters found: " << clusters.size() <<  " filtered clusters: " << filtered_clusters.size() << std::endl;
            for(const auto& cluster: clusters) {
                auto first_pixel = *cluster.begin();
                pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
                bool in_filtered = std::find(filtered_clusters.begin(), filtered_clusters.end(), cluster) != filtered_clusters.end();
                std:: cout << "cluster size: " << cluster.size() << " with color: " << static_cast<int>( color) << " in filtered cluster included " << in_filtered <<  std::endl;
            } 
        }
        // Validate clusters
        for (const auto& cluster : filtered_clusters) {
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
           
            // Check if it's a valid yellow key
            if (size >= 26 && size <= 30 && color_match(color, COLORS.at("yellow"))  && type == "yellow_key")  // Room color check
            {
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
            //changed 25 --> 22
            }else if (size >= 20 && size <= 22 && color_match(color, COLORS.at("yellow")) && type == "yellow_sword") {
                yswrt = true; 
                ykeyt = false;
                bkeyt = false;
                chalicet = false;
                if (printing_debug) std::cout << " founnd yswrt " << std::endl;
                return true; 
            }else if (size >= 66 && size <= 68 && type == "chalice" && check_pattern_chalice(screen_pixels, cluster)) {
                chalicet = true; 
                ykeyt = false;
                bkeyt = false;
                yswrt = false;
                if (printing_debug_adventure) std::cout << " founnd chalicet " << std::endl;
                return true; 
            }
        }
        if (printing_debug||printing) std::cout << " found nothing " << std::endl; 
        
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
        //cluster pixels 
        return cluster_pixels(touching_pixels, screen_pixels);
    }
    bool is_sword_touching_ydragon(const std::vector<pixel_t>& screen_pixels,const std::vector<pixel_t>& prev_image, bool printing = false) const {
        // Get sword location from entire screen detection
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        std::pair<int, int> sword_location = {-1, -1};
        if(printing) std::cout << "Detecting items on screen..." << items.size() << std::endl;
        for (const auto& item : items) {
            if (item.first == "yellow_sword") {
                sword_location = item.second;
                break;
            }
        }
        
        if (sword_location.first == -1) return false; // Sword not found
        //Get ysword cluster
        
        if(printing) std::cout << "sword location: " << sword_location.first << " " << sword_location.second << std::endl;
        std::set<std::pair<int, int>> candidates = {sword_location};
        for(int y = sword_location.second - 1; y <= sword_location.second + 6; y++) {
            for(int x = sword_location.first - 1; x <= sword_location.first + 8; x++) {
                if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) continue;
                pixel_t color = screen_pixels[y * SCREEN_WIDTH + x];
                if (color_match(color, COLORS.at("yellow"))) {
                    candidates.insert({x, y});
                }
            }
        }
         if(printing) std::cout << "sword candidates: " << candidates.size() << std::endl;
        auto sword_cluster = cluster_pixels(candidates, screen_pixels);
         std::set<std::pair<int, int>> sword_cluster_filtered;
        for(auto& cluster : sword_cluster) {
           
             // Calculate cluster size and color
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            if(printing) std::cout << "sword cluster size: " << cluster.size() << " color: " << static_cast<int>(color) << std::endl;
            if (size >= 20 && size <= 22 && color_match(color, COLORS.at("yellow"))) { //22 px
                sword_cluster_filtered = cluster;
                if(printing) std::cout << "sword cluster filtered size: " << sword_cluster_filtered.size() << std::endl;
                break;
            }
        }
        // Get dragon clusters
        auto dragon_clusters = dragon_helper_function(screen_pixels, prev_image);
        
        // Check each dragon cluster for adjacency with sword
        for (const auto& cluster : dragon_clusters) {
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            //changed
            bool size = (cluster.size() >= 142 && cluster.size() < 180); 
            bool pattern_alive = check_dragon_pattern_alive(screen_pixels, cluster, "ydragon");
            bool pattern_firing = check_dragon_pattern_firing(screen_pixels, cluster, "ydragon", printing);
            bool pattern_dead = check_dragon_pattern(screen_pixels, cluster, "ydragon");
            bool pattern = pattern_alive || pattern_firing || pattern_dead;
            // Check if this is a yellow dragon cluster (alive)
            if (color_match(color, COLORS.at("ydragon")) && (pattern || size) ) {
                // Check if any pixel in dragon cluster is adjacent to sword
                if(printing) std::cout << "Checking dragon cluster with size: " << cluster.size() << " and color: " << static_cast<int>(color) << std::endl;
                int adjacent_count = 0;
                for (const auto& dragon_pixel : cluster) {
                    if (sword_cluster_filtered.find(dragon_pixel) != sword_cluster_filtered.end()) {
                        adjacent_count++;
                    }
                }
                if(printing) std::cout << "Adjacent count: " << adjacent_count << std::endl;
                if (adjacent_count > 0) {
                    return true;
                }
            }
        }
        
        return false;
    }
    bool is_sword_touching_gdragon(const std::vector<pixel_t>& screen_pixels, const std::vector<pixel_t>& prev_image, bool printing = false) const {
        // Get sword location from entire screen detection
        auto items = detect_items_entire_screen(screen_pixels, prev_image);
        std::pair<int, int> sword_location = {-1, -1};
        if(printing) std::cout << "Detecting items on screen..." << items.size() << std::endl;
        for (const auto& item : items) {
            if (item.first == "yellow_sword") {
                sword_location = item.second;
                break;
            }
        }
        
        if (sword_location.first == -1) return false; // Sword not found
        //Get ysword cluster
        
        if(printing) std::cout << "sword location: " << sword_location.first << " " << sword_location.second << std::endl;
        std::set<std::pair<int, int>> candidates = {sword_location};
        for(int y = sword_location.second - 1; y <= sword_location.second + 6; y++) {
            for(int x = sword_location.first - 1; x <= sword_location.first + 8; x++) {
                if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) continue;
                pixel_t color = screen_pixels[y * SCREEN_WIDTH + x];
                if (color_match(color, COLORS.at("yellow"))) {
                    candidates.insert({x, y});
                }
            }
        }
        if(printing) std::cout << "sword candidates: " << candidates.size() << std::endl;
        auto sword_cluster = cluster_pixels(candidates, screen_pixels);
        std::set<std::pair<int, int>> sword_cluster_filtered;
        for(auto& cluster : sword_cluster) {
           
             // Calculate cluster size and color
            size_t size = cluster.size();
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            if(printing) std::cout << "sword cluster size: " << cluster.size() << " color: " << static_cast<int>(color) << std::endl;
            if (size >= 20 && size <= 22 && color_match(color, COLORS.at("yellow"))) { //22 px
                sword_cluster_filtered = cluster;
                if(printing) std::cout << "sword cluster filtered size: " << sword_cluster_filtered.size() << std::endl;
                break;
            }
        }
        // Get dragon clusters
        auto dragon_clusters = dragon_helper_function(screen_pixels, prev_image);
        
        // Check each dragon cluster for adjacency with sword
        for (const auto& cluster : dragon_clusters) {
            auto first_pixel = *cluster.begin();
            pixel_t color = screen_pixels[first_pixel.second * SCREEN_WIDTH + first_pixel.first];
            //changed added pattern matching
            bool size = (cluster.size() >= 142 && cluster.size() < 180);
            bool pattern_alive = check_dragon_pattern_alive(screen_pixels, cluster, "gdragon");
            bool pattern_firing = check_dragon_pattern_firing(screen_pixels, cluster, "gdragon", printing);
            bool pattern_dead = check_dragon_pattern(screen_pixels, cluster, "gdragon");
            bool pattern = pattern_alive || pattern_firing || pattern_dead;
            // Check if this is a yellow dragon cluster (alive)
            if (color_match(color, COLORS.at("gdragon")) && (pattern || size) ) {
                // Check if any pixel in dragon cluster is adjacent to sword
                if(printing) std::cout << "Checking dragon cluster with size: " << cluster.size() << " and color: " << static_cast<int>(color) << std::endl;
                int adjacent_count = 0;
                for (const auto& dragon_pixel : cluster) {
                    if (sword_cluster_filtered.find(dragon_pixel) != sword_cluster_filtered.end()) {
                        adjacent_count++;
                    }
                }
                if(printing) std::cout << "Adjacent count: " << adjacent_count << std::endl;
                if (adjacent_count > 0) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    
    //state management functions regarding the nodes 
    void reset_item_state() const{
        ykeyt = false;
        bkeyt = false;
        yswrt = false;
        chalicet = false;
        //changed removed ydragon and gdragon getting reseted
        Last_room_color = -1;
    }
    void set_item_state(Node* node, bool printing = false) const {
        
        current_node = node;
        ykeyt = node->node_ykeyt;
        bkeyt = node->node_bkeyt;
        yswrt = node->node_yswrt;
        chalicet = node->node_chalicet;
        ydragon = node->node_ydragon;
        gdragon = node->node_gdragon;
        Last_room_color = node->node_Last_room_color;
        if(printing){
            std::cout<< "Setting item state for node with action: " << node->action_
            << " ykeyt:" << node->node_ykeyt
            << " bkeyt:" << node->node_bkeyt
            << " yswrt:" << node->node_yswrt
            << " chalicet:" << node->node_chalicet
            << " ydragon:" << node->node_ydragon
            << " gdragon:" << node->node_gdragon
            << " Last_room_color:" << node->node_Last_room_color
            << std::endl;
        }
        //if(printing) std::cout << "current node action: " << current_node->action_ << " && the best_node action is" << node->action_ << std::endl;
    }
    void setting_node_state(Node* node, bool printing = false) const {
        node->node_ykeyt = ykeyt;
        node->node_bkeyt = bkeyt;
        node->node_yswrt = yswrt;
        node->node_chalicet = chalicet;
        node->node_ydragon = ydragon;
        node->node_gdragon = gdragon;
        node->node_Last_room_color = Last_room_color;
         if(printing){
            std::cout<< "Setting node state for node with action: " << node->action_
            << " ykeyt:" << node->node_ykeyt
            << " bkeyt:" << node->node_bkeyt
            << " yswrt:" << node->node_yswrt
            << " chalicet:" << node->node_chalicet
            << " ydragon:" << node->node_ydragon
            << " gdragon:" << node->node_gdragon
            << " Last_room_color:" << node->node_Last_room_color
            << std::endl;
        }
        
    }
    void setting_dist_state_from_root(Node* node) const {
        if (node == nullptr) return;
        Node* temp = node; 
        if(node->parent_ != nullptr) temp = node->parent_;
        root_dist_to_key = ykey_dist(node->screen_pixels_, temp->screen_pixels_);
        root_dist_to_chalice = chalice_dist(node->screen_pixels_, temp->screen_pixels_);
        root_dist_to_bkey = bkey_dist(node->screen_pixels_, temp->screen_pixels_);
        root_dist_to_sword = ysword_dist(node->screen_pixels_, temp->screen_pixels_);

    }

    std::vector<bool> check_sketches_preconditions (const std::vector<pixel_t>& pre, const std::vector<pixel_t>& post, const SimPlanner& planner, Node* node = nullptr, bool printing = false, bool is_root = false) const{
            std::vector<bool> sketches_pre(planner.sketches_.size(), false);
            if( node != nullptr) set_item_state(node, printing);
            //changed 
            //if(is_root) setting_dist_state_from_root(node);
            if(printing) printing_sketches_ = true;
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
            printing_sketches_ = false;
            if( node != nullptr) setting_node_state(node, printing);
            return sketches_pre;
    }
    std::vector<bool> check_sketches_goals(const std::vector<pixel_t>& pre, const std::vector<pixel_t>& post, const std::vector<pixel_t>& prevs, const SimPlanner& planner,   Node* node = nullptr, bool printing = false) const {
        std::vector<bool> sketches_post(sketches_.size(), false);
        // Check current priority sketch
        if( node != nullptr) set_item_state(node, printing);
        if (printing) printing_sketches_ = true;
        for(size_t i = 0; i < planner.sketches_.size(); ++i) {
            sketches_post[i] = planner.sketches_[i].goal(planner, pre, post, prevs);
        }
        if(printing) {
                std::cout << "Sketches post: ";
                for (const auto& sketch : sketches_post) {
                    std::cout << sketch << " ";
                }
                std::cout << std::endl;
        }
        if( node != nullptr) setting_node_state(node,printing);
        printing_sketches_ = false;
        return sketches_post;
    }
    void initalize_sketches_adventure() {
        sketches_.clear();

        // Sketch 0: Acquire yellow key
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
             ////planner.calculate_distance_from_goal(curr);
                if(printing_sketches_) std::cout << "SKETCH 0 PRE Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions); 
                bool room = planner.ykeyr(curr, prev);
                bool sword_room = planner.yswr(curr, prev);
                
                bool cond =  (!key && room && !sword_room )  ;  //D == 1 &&
                if(printing_sketches_){
                std::cout<< std::endl; 
                std::cout << "SKETCH 0 PREc (Acquire Key):" //<< D 
                        << " | !ykey=" << !key
                        << " | ykeyr=" << room << " | !sword_room " << !sword_room
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 0 GOAL Computation " << std::endl;
                bool key = planner.ykey(curr,prev, planner.printing_sketches_functions);
                bool goal_achieved =  key; //&& D==1;
                ////planner.calculate_distance_from_goal(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 0 GOAL: " << (goal_achieved ? "ACHIEVED" : "IN PROGRESS")
                        << " | ykey=" << key 
                        //<< " | ykey_dist: " << curr_dist << " (prev: " << prev_dist << ")"
                        //<< " | D=" << D 
                        << std::endl;
                 }
                return goal_achieved;
            },
            "Acquire yellow key"
        });
        // Sketch 1: Navigate to sword room 
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 1 PRE Computation " << std::endl;
                
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                //planner.calculate_distance_from_goal(curr);
                bool room = planner.yswr(curr, prev);
                bool cond =  key &&  !room; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 1 PRE go sword room:"  
                << " | ykey=" << key << " | !ysword room=" << !room
                        << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 1 GOAL Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr, prev);
                //planner.calculate_distance_from_goal(curr);
                bool goal_achieved =  room && key; 
                if(printing_sketches_){
                std::cout << "SKETCH 1 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") << " | yswr=" << room << " | ykey=" << key
                        << std::endl;
                }
                return goal_achieved;
            },
            "Reach yswr room"
        });
        /* //drop ykey
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                 if(printing_sketches_) std::cout << "SKETCH 2 PRE Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr);
                int dist = root_dist_to_sword;//planner.ysword_dist(curr);
                bool cond = room && key ; //&& dist <= 25 && dist > 0; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 2 PRE Drop key: " 
                << " | ykey=" << key << " | ysw room=" << room 
                << " | dist = " << dist
                << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 2 GOAL Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr);
                //int dist = planner.ysword_dist(curr);
                bool goal_achieved =  !key  && room; //&& dist <= 25 && dist > 0; //&& D==1;
                if(printing_sketches_){
                std::cout << "SKETCH 2 GOAL (dropping key): " << (goal_achieved ? "REACHED" : "MOVING") 
                << " | ykey=" << !key
                        << " | ykey room= " << room << std::endl;
                }
                return goal_achieved;
            },
            "Drop yellow key"
        });*/
        //move away from key and towards the sword
        /*sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                 if(printing_sketches_) std::cout << "SKETCH 3 PRE Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr);
                int dist = planner.ysword_dist(curr);
               
                //75 as this the org distance when entering the room 
                bool cond = room && !key && abs(75-dist) <= 5 &&  !sword; //D == 1  &&
                if(cond) {
                    planner.root_dist_to_key = planner.ysword_dist(curr);
                    planner.root_dist_to_sword = planner.ysword_dist(curr);
                }
                if(printing_sketches_){
                std::cout << "SKETCH 3 PRE Move away: " 
                << " | ykey=" << key << " | ysw room=" << room 
                //<< " | dist=" << dist
                <<" | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 3 GOAL Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr);
                int ysword_dist = planner.ysword_dist(curr);
                int ykey_dist = planner.ykey_dist(curr);
                bool closer = (ysword_dist > 0 && ykey_dist > 0 && ysword_dist < ykey_dist-20);
                bool goal_achieved =  !key  && room &&  closer; //abs(root_dist_to_sword-dist) >= 15; //&& D==1;
                if(printing_sketches_){
                std::cout << "SKETCH 3 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") 
                << " | ykey=" << !key
                << " | ykey_dist = " << ykey_dist << " root_dist_to_sword = " << planner.root_dist_to_sword 
                << " | ysword_dist = " << ysword_dist << " root_dist_to_key = " << planner.root_dist_to_key
                << " | ykey room= " << room << std::endl;
                }
                if(goal_achieved){
                   if(printing_sketches_) planner.printing_screen(curr);
                }
                return goal_achieved;
            },
            "Move away with ykey"
        });
       */
        //get ysword
         sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 2 PRE Computation " << std::endl;
                bool key = planner.ykey(curr,prev,planner.printing_sketches_functions);
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool room = planner.yswr(curr, prev);
                //planner.calculate_distance_from_goal(curr);
                //int ysword_dist = planner.ysword_dist(curr);
                //int ykey_dist = planner.ykey_dist(curr);
                //bool closer = (ysword_dist > 0 && ykey_dist > 0 && ysword_dist < ykey_dist-25);
                //changed !key --> key
                bool cond = room && !sword  && key; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 2 PRE: (get ysword)" 
                << " | !ykey=" << !key << " | ysw room=" << room << " | !ysword=" << !sword
                //<< " | ykey_dist = " << ykey_dist << " root_dist_to_sword = " << planner.root_dist_to_sword 
                //<< " | ysword_dist = " << ysword_dist << " root_dist_to_key = " << planner.root_dist_to_key
                << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 2 GOAL Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                //planner.calculate_distance_from_goal(curr);
                 //bool key = planner.ykey(curr,prev,printing_sketches_);
                bool goal_achieved =  sword ; //&& !key; //&& D==1;
                if(printing_sketches_){
                std::cout << "SKETCH 2 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") <<  " | ysword=" << sword
                    << std::endl;
                }
                //if(goal_achieved) std::cout << "SKETCH 4 GOAL REACHED " << current_node->action_ << std::endl;
                //<< " | ysword_dist: " << dist_curr << " (prev: " << dist_prev << ")" 
                return goal_achieved;
            },
            "Pick up yellow sword"
        });
        //reach ydragon_room
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 3 PRE Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool ydrag = planner.ydragon_killed(curr, prev, printing_sketches_);
                //planner.calculate_distance_from_goal(curr);
                bool ydrag_in_room = planner.ydragonr(curr,prev,  printing_sketches_);
                bool cond = sword && !ydrag_in_room && !ydrag; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 3 PRE:" << " | ysword=" << sword << " | ydrag_in_room=" << ydrag_in_room << " | " << " !ydrag=" << !ydrag << " |" << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 3 GOAL Computation " << std::endl;
                //planner.calculate_distance_from_goal(curr);
                bool sword = planner.ysword(curr,prev,printing_sketches_functions);
                bool ydrag_in_room = planner.ydragonr(curr,prev, printing_sketches_);
                bool goal_achieved = sword && ydrag_in_room;
                if(printing_sketches_){
                std::cout << "SKETCH 3 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") <<  " | ysword=" << sword  << " | ydragon_in room=" << ydrag_in_room << std::endl;
                }
               
                return goal_achieved;
            },
            "reach dragon room"
        });
        //kill ydragon
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 4 PRE Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,printing_sketches_functions);
                bool ydrag = planner.ydragon_killed(curr,prev, printing_sketches_);
                bool ydrag_in_room = planner.ydragonr(curr,prev,  printing_sketches_); 
                bool cond = sword && !ydrag && ydrag_in_room;
                //planner.calculate_distance_from_goal(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 4 PRE:" << " | ysword=" << sword << " | !ydrag=" << !ydrag << " ydrag_in_room" << ydrag_in_room << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 4 GOAL Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool ydrag = planner.ydragon_killed(curr, prev, printing_sketches_);
                bool ydrag_in_room = planner.ydragonr(curr, prev,  printing_sketches_);
                //planner.calculate_distance_from_goal(curr);
                int curr_dist = planner.sword_dist_to_ydragon(curr, prev);
                int prev_dist = planner.sword_dist_to_ydragon(prevs, prev); 
                bool sword_dragon = planner.is_sword_touching_ydragon(curr,prev, printing_sketches_);
                bool dist =  (curr_dist < prev_dist - 30 && curr_dist >= 0 && prev_dist >= 0);
                bool kill_dragon = (ydrag || sword_dragon ); //changed || dist || dist
                bool goal_achieved = sword && kill_dragon && ydrag_in_room;
                
                if(printing_sketches_){
                std::cout << "SKETCH 4 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") 
                <<  " | ysword=" << sword  
                << " | ydragon_killed=" << ydrag << "  or sword_dragon=" << sword_dragon
                << " | ydragon_distance=" << dist << " curr_dist=" << curr_dist << " prev_dist=" << prev_dist
                << " | what exactly curr: " << (curr_dist >= 0 ? "all_found" : (curr_dist == -1 ? "sword not found" : (curr_dist == -2 ? "dragon not found" : (curr_dist == -3 ? "highlight cube not found" : "unknown"))))
                 << " | what exactly curr: " << (curr_dist >= 0 ? "all_found" : (curr_dist == -1 ? "sword not found" : (curr_dist == -2 ? "dragon not found" : (curr_dist == -3 ? "highlight cube not found" : "unknown"))))
                << " | ydragon_in_room=" << ydrag_in_room << std::endl;
                }
                return goal_achieved;
            },
            "kill ydragon"
        });
        //reach green_dragon_room
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 5 PRE Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                //planner.calculate_distance_from_goal(curr);
                bool gdrag_in_room = planner.gdragonr(curr, prev);
                bool ydrag = planner.ydragon_killed(curr, prev, printing_sketches_);
                bool cond = sword && ydrag && !gdrag_in_room;
                if(printing_sketches_){
                std::cout << "SKETCH 5 PRE:" << " | ysword=" << sword << " | !gdrag_in_room=" << !gdrag_in_room << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 5 GOAL Computation " << std::endl;
                //planner.calculate_distance_from_goal(curr);
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool gdrag_in_room = planner.gdragonr(curr,prev, printing_sketches_);
                bool bkey = planner.bkey(curr,prev,printing_sketches_);
                bool goal_achieved = sword && (gdrag_in_room ); //added bkey as alternative goal || bkey;
                if(printing_sketches_){
                std::cout << "SKETCH 5 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") <<  " | ysword=" << sword  << " | gdragon_in room=" << gdrag_in_room << " |bkey " << bkey << std::endl;
                }
               
                return goal_achieved;
            },
            "reach d"
        });
        //kill gdragon
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 6 PRE Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,printing_sketches_);
                bool ydrag = planner.ydragon_killed(curr, prev, printing_sketches_);
                bool gdrag_in_room = planner.gdragonr(curr, prev,  printing_sketches_); 
                bool gdrag = planner.gdragon_killed(curr, prev,  printing_sketches_);
                bool cond = sword && ydrag && gdrag_in_room && !gdrag;
                //planner.calculate_distance_from_goal(curr);
                if(printing_sketches_){
                std::cout << "SKETCH 6 PRE:" << " | ysword=" << sword << " | ydrag=" << ydrag << " | gdrag_in_room=" << gdrag_in_room << " | !gdrag=" << !gdrag << " | " << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 6 GOAL Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,printing_sketches_);
                bool gdrag = planner.gdragon_killed(curr, prev,  printing_sketches_);
                bool gdrag_in_room = planner.gdragonr(curr,prev,   printing_sketches_);
                //planner.calculate_distance_from_goal(curr);
                bool sword_dragon = planner.is_sword_touching_gdragon(curr, prev,  printing_sketches_);
                
                bool kill_dragon = (gdrag || sword_dragon); //changed || dist 
                bool goal_achieved = sword && kill_dragon && gdrag_in_room;
                
                if(printing_sketches_){
                std::cout << "SKETCH 6 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") 
                <<  " | ysword=" << sword  
                << " | gdragon_killed=" << gdrag << "  or sword_dragon=" << sword_dragon
                << " | gdragon_in_room=" << gdrag_in_room << std::endl;
                }
                return goal_achieved;
            },
            "kill gdragon"
        });
        //get bkey
         sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 7 PRE Computation " << std::endl;
                bool key = planner.bkey(curr,prev,printing_sketches_);
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool room = planner.bkeyr(curr,prev);
                bool gdrag = planner.gdragon_killed(curr, prev, printing_sketches_);
                bool cond = room && sword && !key ; //D == 1  && && gdrag
                if(printing_sketches_){
                std::cout << "SKETCH 7 PRE: (get bkey)" 
                << " | !bkey=" << !key << " | bkey room=" << room << " | ysword=" << sword
                << " | gdrag=" << gdrag << " | " 
                << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 7 GOAL Computation " << std::endl;
                bool key = planner.bkey(curr,prev,planner.printing_sketches_);
                bool goal_achieved =  key ; //&& !key; //&& D==1;
                if(printing_sketches_){
                std::cout << "SKETCH 7 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") <<  " | bkey=" << key << std::endl;
                }
                //if(goal_achieved) std::cout << "SKETCH 4 GOAL REACHED " << current_node->action_ << std::endl;
                //<< " | ysword_dist: " << dist_curr << " (prev: " << dist_prev << ")" 
                return goal_achieved;
            },
            "Pick up black key"
        });
          //reach ydragon_room_with bkey
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 8 PRE Computation " << std::endl;
                bool sword = planner.ysword(curr,prev,planner.printing_sketches_functions);
                bool ydrag = planner.ydragon_killed(curr, prev,  printing_sketches_);
                bool key = planner.bkey(curr,prev,printing_sketches_);
                //planner.calculate_distance_from_goal(curr);
                bool ydrag_in_room = planner.ydragonr(curr, prev,  printing_sketches_);
                bool cond = !ydrag_in_room && ydrag && key; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 8 PRE:" << " | ysword=" << sword << " | ydrag_in_room=" << ydrag_in_room << " | " << " !ydrag=" << !ydrag << " |" <<  " bkey"<< key << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 8 GOAL Computation " << std::endl;
                //planner.calculate_distance_from_goal(curr);
                bool key = planner.bkey(curr,prev,printing_sketches_);
                bool ydrag_in_room = planner.ydragonr(curr, prev,  printing_sketches_);
                bool goal_achieved = key && ydrag_in_room;
                if(printing_sketches_){
                std::cout << "SKETCH 8 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") <<  " | bkey=" << key  << " | ydragon_in room=" << ydrag_in_room << std::endl;
                }
               
                return goal_achieved;
            },
            "reach dragon room with bkey"
        });
        //reach chalice room 
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr) {
                if(printing_sketches_) std::cout << "SKETCH 9 PRE Computation " << std::endl;
                bool ydrag = planner.ydragon_killed(curr, prev,  printing_sketches_);
                bool key = planner.bkey(curr,prev,printing_sketches_);
                bool chalice = planner.chalicer(curr, prev);
                bool ydrag_in_room = planner.ydragonr(curr, prev, printing_sketches_);
                bool cond = ydrag_in_room && ydrag && key && !chalice; //D == 1  &&
                if(printing_sketches_){
                std::cout << "SKETCH 9 PRE:"  << " | ydrag_in_room=" << ydrag_in_room << " | " << " ydrag=" << ydrag << " |" <<  " bkey="<< key << " | chalicer=" << chalice << " |" << (cond ? "ACTIVE" : "INACTIVE") << std::endl;
                }
                return cond;
            },
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev, const std::vector<pixel_t>& curr, const std::vector<pixel_t>& prevs) {
                if(printing_sketches_) std::cout << "SKETCH 9 GOAL Computation " << std::endl;
                //planner.calculate_distance_from_goal(curr);
                bool key = planner.bkey(curr,prev,printing_sketches_);
                bool ydrag_in_room = planner.ydragonr(curr, prevs, printing_sketches_);
                bool chalice = planner.chalicer(curr,prevs,  printing_sketches_);
                auto temp =  planner.regions_for_cube(curr);
                bool reached = (planner.Last_room_color == 11); //chalice 
                bool goal_achieved = key && !ydrag_in_room && reached; 
                if(printing_sketches_){
                std::cout << "SKETCH 9 GOAL: " << (goal_achieved ? "REACHED" : "MOVING") 
                <<  " | bkey=" << key  << " | !ydragon_in room=" << !ydrag_in_room << " | chalicer=" << chalice << " | last_room_color=" << planner.Last_room_color
                << " | reached=" << reached << std::endl;
                }
               
                return goal_achieved;
            },
            "reach dragon room with bkey"
        });
    }

    void initialize_sketches_seaquest() {
        //P: Person on Board, H: Human in Water, E: enemies, O:Oxygen
        sketches_.clear();
        //Sketch 1: not A → B (no oxygen → get oxygen )
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_) {
                //std::cout << "sketch 1 pre" << std::endl; 
                //!planner.has_oxygen(curr)
                return planner.oxygen_left(curr) <= 3 ;
            },
            [this](const SimPlanner& planner,const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_, const std::vector<pixel_t>& prevs= MyALEScreen::initial_background_image_) {
                /*
                auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                auto[past_enemies, past_human] = planner.has_enemies(prev); 
                bool enemies_regardless = (curr_enemies >= past_enemies ) || (curr_enemies <= past_enemies ); 
                bool human_in_water_regardless = (curr_human >= past_human ) || (curr_human <= past_human ); 
                && enemies_regardless && human_in_water_regardless */
                return planner.oxygen_left(curr) > 3 ;//&& planner.count_humans_on_board(prev) <= planner.count_humans_on_board(curr) ; 
            },
            "no O → O, P?, E?, H?"
        });
    
        // Sketch 2: P<6, O → O, P++ increases (not max human onboard → humans on board increases)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_) {
                //std::cout << "sketch 2 pre" << std::endl;
                 auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                return planner.has_oxygen(curr) && planner.count_humans_on_board(curr) < 6 && curr_human > 0 ;
            },
            [this](const SimPlanner& planner,const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_, const std::vector<pixel_t>& prevs= MyALEScreen::initial_background_image_) {
                
                auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                auto[past_enemies, past_human] = planner.has_enemies(prevs); 
                /*
                && enemies_regardless
                bool enemies_regardless = (curr_enemies >= past_enemies ) || (curr_enemies <= past_enemies ); 
                */
                
                return planner.has_oxygen(curr) &&  planner.count_humans_on_board(prevs) < planner.count_humans_on_board(curr)  && curr_human <= past_human; 
            },
            "P<6 and O and H>0 → P++, O, H--"
        });
    
        // Sketch 3: O && P == 6 → O && P == 0 (oxygen and max #ppl in ship ->  set number of ppl in ship to 0)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_) {
                //std::cout << "sketch 3 pre" << std::endl;
                return planner.has_oxygen(curr) && planner.count_humans_on_board(curr) == 6;
            },
            [this](const SimPlanner& planner,const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_, const std::vector<pixel_t>& prevs= MyALEScreen::initial_background_image_) {
               
                /*
                 auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                auto[past_enemies, past_human] = planner.has_enemies(prev);
                 && curr_enemies >= past_enemies && curr_human >= past_human
                */ 
                return planner.has_oxygen(curr) && planner.count_humans_on_board(curr) == 0 ; 
            },
            " O && P == 6 → O && P == 0 "
        });
        // Sketch 4: E > 0 && O --> E-- (enemies present &&  oxygen → enemies less)
        sketches_.push_back(Sketch{
            [this](const SimPlanner& planner, const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_) {
                //std::cout << "sketch 4 pre" << std::endl;
                auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                return planner.has_oxygen(curr) && curr_enemies > 0;
            },
            [this](const SimPlanner& planner,const std::vector<pixel_t>& prev= MyALEScreen::initial_background_image_, const std::vector<pixel_t>& curr = MyALEScreen::initial_background_image_, const std::vector<pixel_t>& prevs= MyALEScreen::initial_background_image_) {
                auto[curr_enemies, curr_human] = planner.has_enemies(curr); 
                auto[past_enemies, past_human] = planner.has_enemies(prev); 
                //&& planner.count_humans_on_board(prev) <= planner.count_humans_on_board(curr)
                return planner.has_oxygen(curr)  && curr_enemies >= past_enemies && curr_human >= past_human ;
            },
            " E > 0 && O --> E-- && O && H? "
        });

    }
    
    //sequest 
    //const std::vector<bool> humans = {false,false, false, false,false,false}; 
    const pixel_t SHIP_COLOR   = greyscale(187,187, 53);
    const pixel_t WATER_COLOR  = greyscale(0, 28, 136);
    const pixel_t HUMAN_COLOR = greyscale(24,26,167);//greyscale(18,19,157); 
    const pixel_t HUMAN_IN_WATER = greyscale(66,72,200);
    const pixel_t black = greyscale(0,0,0);
    const std::vector<int> human_positions = { 300, 340, 380, 420, 460, 500 }; // 
    int count_humans_on_board(const std::vector<pixel_t>& screen_pixels) const {
       
        // Define your screenshot resolution (480x640 as an example, adjust as necessary)
       
        int human_count = 0;
        size_t y_min = 180; 
        size_t y_max = 185; 
        for(int i = 0; i < human_positions.size(); i++) {
            for(size_t j = y_min; j < y_max; j++ ){
                int index = j* SCREEN_WIDTH + static_cast<size_t>(human_positions[i] * SCALE_X); 
                bool not_background = color_diff(screen_pixels[index], MyALEScreen::initial_background_image_[index]) > COLOR_THRESHOLD; 
                bool human_color = color_diff(screen_pixels[index], HUMAN_COLOR) <= COLOR_THRESHOLD; 
                if(not_background && human_color ){
                        human_count++;
                        break; 
                } // Calculate the pixel index in the 1D array
            }
        }
        if (human_count > 0)  {
            //std::cout << "Humans on board: " << human_count << std::endl;
            /*for(int i = 0; i < screen_pixels.size(); i++) {
                std::cout << static_cast<int>( screen_pixels[i]) << " ";
                if(i != 0 && i % SCREEN_WIDTH == 0 ) std::cout << std::endl; 
            }
            std::cout << std::endl; */
        }
        //std::cout << "Humans on board: " << human_count << std::endl;
       return human_count;
    }
    bool has_oxygen(const std::vector<pixel_t> &feature_atoms) const {
        
        // Define your screenshot resolution (480x640 as an example, adjust as necessary)
        //for (int y = static_cast<int>(475 * SCALE_Y); y <= static_cast<int>(500*SCALE_Y); y++) {
            for (int x = static_cast<int>(230* SCALE_X); x <= static_cast<int>(560 * SCALE_X); x++) {
                int pixel_index = static_cast<int>(489 * SCALE_Y) * SCREEN_WIDTH + x;  // Calculate the pixel index in the 1D array
                bool is_black_pixel = color_diff(feature_atoms[pixel_index], black) <= COLOR_THRESHOLD ;
                bool is_background  = color_diff(feature_atoms[pixel_index], MyALEScreen::initial_background_image_[pixel_index]) <= COLOR_THRESHOLD;
                if (is_black_pixel && !is_background) {
                    //std::cout << "Black pixel found in oxgen bar" << std::endl;
                    return false;  // Found a non-empty pixel, indicating oxygen presence
                }
            }
        //}
    
        //std::cout << "No black pixel found in oxygen bar" << std::endl;
        return true;

    }
    int oxygen_left(const std::vector<pixel_t> &feature_atoms) const {
        int resultpercent = 0; 
        for(size_t i = 49; i < 111; i += 6){
            int pixel_index = 172 * SCREEN_WIDTH + i;  // Calculate the pixel index in the 1D array
            if(color_diff(feature_atoms[pixel_index], 214) <= COLOR_THRESHOLD) resultpercent ++; 
        }
        return resultpercent; 
    }
    std::tuple<int, int> has_enemies(const std::vector<pixel_t> &feature_atoms) const {
        const int scan_y_min  = 158; 
        const int scan_y_max  = 435;
        const int scan_x_min  = 35;
        const int scan_x_max  = 800;
        int enemies = 0; 
        int human = 0; 
        for (int y = static_cast<int>(scan_y_min * SCALE_Y); y <= static_cast<int>(scan_y_max * SCALE_Y); ++y) {
            for (int x = static_cast<int>(scan_x_min * SCALE_X); x <= static_cast<int>(scan_x_max * SCALE_X); ++x) {
                int pixel_index = y * SCREEN_WIDTH + x;
                //check if pixel is water, ship, or human --> skip 
                //if not --> check same as background  --> skip
                //if not --> count as enemy
                //logic: not same as background 
                //bool is_water = color_diff(feature_atoms[pixel_index], WATER_COLOR) <= COLOR_THRESHOLD;
                bool is_ship = color_diff(feature_atoms[pixel_index], SHIP_COLOR) <= COLOR_THRESHOLD;
                bool is_human = color_diff(feature_atoms[pixel_index], HUMAN_IN_WATER) <= COLOR_THRESHOLD;
                bool is_background = color_diff(feature_atoms[pixel_index], MyALEScreen::initial_background_image_[pixel_index]) <= COLOR_THRESHOLD;
                if( !(is_ship || is_human || is_background)  ) {
                    //std::cout << "Skipping water, ship, or human pixel at (" << x << ", " << y << ")" << std::endl;
                    enemies ++; // Skip water, ship, or human pixels
                }else if (is_human) human++;
            }
        }
        return std::make_tuple(enemies, human);
    }
    
    
    
};
#endif

