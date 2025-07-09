// (c) 2017 Blai Bonet

#ifndef SCREEN_H
#define SCREEN_H

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include  "/usr/local/include/ale/ale_interface.hpp"

#include "logger.h"
#include "utils.h"
using namespace ale;

struct MyALEScreen {
    const int type_; // type=0: no features, type=1: basic features, type=2: basic + B-PROS, type=3: basic + B-PROS + B-PROT type 4 = Breakout
    const ALEScreen &screen_;
    const size_t width_;
    const size_t height_ ; 
    std::vector<bool> basic_features_bitmap_;
    std::vector<bool> bpros_features_bitmap_;
    std::vector<bool> bprot_features_bitmap_;
    std::vector<bool> breakout_features_bitmap_;
    static const int SCREENSHOT_HEIGHT = 600;
    static const int SCREENSHOT_WIDTH = 800;
    


    //static const size_t width_ = 160;
    //static const size_t height_ = 210; //210
    static const size_t num_basic_features_ = 16 * 14 * 128; // 28,672
    static const size_t num_bpros_features_t0_ = 6803136; // (dc,dr,k1,k2) where k1 < k2, number equal to 31 * 27 * 128 * 127 / 2
    static const size_t num_bpros_features_t1_ = 53504; // (dc,dr,k,k) where dc != 0 or dr != 0, number equal to (31 * 27 - 1) * 128 / 2
    static const size_t num_bpros_features_t2_ = 128; // (dc,dr,k,k) where dc = dr = 0, number equal to 128
    static const size_t num_bpros_features_ = num_bpros_features_t0_ + num_bpros_features_t1_ + num_bpros_features_t2_; // 6,856,768
    static const size_t num_bprot_features_ = 31 * 27 * 128 * 128; // 13,713,408
    static const size_t num_adventure_features_ = 10+4*75; //90
    static const size_t adventure_base_ = num_basic_features_ + num_bpros_features_ + num_bprot_features_;
    std::vector<bool> adventure_features_bitmap_;
    static const size_t adventure_room_start_ = adventure_base_;
    static const size_t adventure_ykey_start_ = adventure_base_ + 10;
    static const size_t adventure_bkey_start_ = adventure_base_ + 85;
    static const size_t adventure_ysword_start_ = adventure_base_ + 160;
    static const size_t adventure_chalice_start_ = adventure_base_ + 235;
    mutable int Last_room_color = 1; 
    //@todo: need to calculate this number
    /*  // Add brick grid parameters
        static const size_t breakout_min_x_ = 40*SCALE_X;
        static const size_t breakout_max_x_ = 200*SCALE_X;
        static const size_t breakout_min_y_ = 50*SCALE_Y;
        static const size_t breakout_max_y_ = 100*SCALE_Y;
        static const size_t breakout_brick_width_ =40   ;
        static const size_t breakout_brick_height_ = 12;

        // Calculate number of bricks (features)
        static const size_t num_breakout_x_  = (breakout_max_x_ - breakout_min_x_) / breakout_brick_width_;
        static const size_t num_breakout_y_ = (breakout_max_y_ - breakout_min_y_) / breakout_brick_height_;
        static const size_t num_breakout_features_ = num_breakout_x_ * num_breakout_y_;*/
    // Scaling factors based on the screenshot resolution
     const float SCALE_X = static_cast<float>(width_) / SCREENSHOT_WIDTH;
     const float SCALE_Y = static_cast<float>(height_) / SCREENSHOT_HEIGHT;
    static const size_t num_breakout_features= 40; //5 row * 8 column
    static  std::vector<pixel_t> initial_background_image_;
    static size_t num_initial_background_pixels_;

    static ActionVect minimal_actions_;
    static size_t minimal_actions_size_;
    static std::vector<pixel_t> background_;
    static size_t num_background_pixels_;
    
    const bool printing_debug = false; // Set to true to print debug information
    MyALEScreen(ALEInterface &ale,
                int type,
                std::vector<int> *screen_state_atoms = nullptr, std::vector<pixel_t> *screen_pixels = nullptr,
                const std::vector<int> *prev_screen_state_atoms = nullptr)
      : type_(type),
        screen_(ale.getScreen()) , height_(screen_.height()), width_(screen_.width()){
       
        logging::Logger::DebugMode(-100)
          << "screen:"
          << " type=" << type_
          << ", height=" << screen_.height() << " (expecting " << 210 << ")" // for some reason static const int height_ not working here...
          << ", width=" << screen_.width() << " (expecting " << 160 << ")" // for some reason static const int width_ not working here...
          << std::endl;
         
        assert((width_ == screen_.width()) && (height_ == screen_.height()));
        screen_pixels->resize(width_ * height_);
        ale.getScreenGrayscale(*screen_pixels);
        
        //screen_pixels->resize(width_ * height_);
        //fill_image(ale, *screen_pixels);
        /*std::cout << "screen pixels size " << screen_pixels->size() << std::endl; 
        for(int i = 0; i < screen_pixels->size(); ++i) {
           std::cout << static_cast<int>((*screen_pixels)[i]) << " ";
           if(i % width_ == 0) std::cout << std::endl;
        }
        std::cout << std::endl;*/
        
       
        compute_features(ale,type, screen_state_atoms, prev_screen_state_atoms);
    }

    static Action random_action() {
        return minimal_actions_[lrand48() % minimal_actions_size_];
    }
    static void reset(ALEInterface &ale) {
        ale.reset_game();
        for( size_t k = 0; k < 100; ++k )
            ale.act(random_action());
    }
    static void fill_image(ALEInterface &ale, std::vector<pixel_t> &image) {
        const ALEScreen &screen = ale.getScreen();
        size_t width_ = screen.width();
        size_t height_ = screen.height(); 
        for( size_t c = 0; c < width_; ++c ) {
            for( size_t r = 0; r < height_; ++r ) {
                image[r * width_ + c] = screen.get(r, c);
            }
        }
    }
    static void create_background_image(ALEInterface &ale) {
        size_t width_ = ale.getScreen().width();
        size_t height_ = ale.getScreen().height();
        background_ = std::vector<pixel_t>(width_ * height_, 0);
        num_background_pixels_ = width_ * height_;
        initial_background_image_ = std::vector<pixel_t>(width_ * height_, 0);
        num_initial_background_pixels_ = width_ * height_;
    }
    static void compute_background_image(ALEInterface &ale, size_t num_frames) {
        //std::cout << ale.getScreen().width() << " " << ale.getScreen().height() << std::endl;
        size_t width_ = ale.getScreen().width();
        size_t height_ = ale.getScreen().height();
        assert((width_ == ale.getScreen().width()) && (height_ == ale.getScreen().height()));
        float start_time = Utils::read_time_in_seconds();

        logging::Logger::Debug
          << logging::Logger::green()
          << "screen: computing background image (#frames=" << num_frames << ") ... "
          << logging::Logger::normal()
          << std::flush;

        minimal_actions_ = ale.getMinimalActionSet();
        minimal_actions_size_ = minimal_actions_.size();

        std::vector<bool> is_background(width_ * height_, true);
        std::vector<pixel_t> reference_image(width_ * height_);
        std::vector<pixel_t> image(width_ * height_);

        reset(ale);
        fill_image(ale, reference_image);
        int frameskip = ale.getInt("frame_skip");
        for( size_t k = 0; k < num_frames; k += frameskip ) {
            if( ale.game_over() ) reset(ale);
            fill_image(ale, image);
            for( size_t c = 0; c < width_; ++c ) {
                for( size_t r = 0; r < height_; ++r ) {
                    if( (reference_image[r * width_ + c] != image[r * width_ + c]) && is_background[r * width_ + c] ) {
                        is_background[r * width_ + c] = false;
                        --num_background_pixels_;
                    }
                }
            }
            ale.act(random_action());
        }

        for( size_t c = 0; c < width_; ++c ) {
            for( size_t r = 0; r < height_; ++r ) {
                if( is_background[r * width_ + c] ) {
                    background_[r * width_ + c] = reference_image[r * width_ + c];
                }
            }
        }

        float elapsed_time = Utils::read_time_in_seconds() - start_time;

        logging::Logger::DebugMode(0, true)
          << logging::Logger::green()
          << "done in " << elapsed_time << " seconds"
          << logging::Logger::normal()
          << std::endl;
        logging::Logger::Debug 
          << logging::Logger::green()
          << "background: #pixels=" << num_background_pixels_ << "/" << width_ * height_
          << logging::Logger::normal()
          << std::endl;
        
    }
    static void ammend_background_image(ALEInterface &ale,size_t r, size_t c) {
        size_t width_ = ale.getScreen().width();
        size_t height_ = ale.getScreen().height();
        assert(background_[r * width_ + c] > 0);
        assert(num_background_pixels_ > 0);
        background_[r * width_ + c] = 0;
        --num_background_pixels_;
        logging::Logger::DebugMode(-100)
          << "background: #pixels=" << num_background_pixels_ << "/" << width_ * height_
          << std::endl;
    }

    const ALEScreen& get_screen() const {
        return screen_;
    }

    void compute_features(ALEInterface &ale, int type, std::vector<int> *screen_state_atoms, const std::vector<int> *prev_screen_state_atoms) {
        int num_basic_features = 0;
        int num_bpros_features = 0;
        int num_bprot_features = 0;
        int num_breakout_features = 0;
        if( type_ > 0 ) {
            basic_features_bitmap_ = std::vector<bool>(num_basic_features_, false);
            compute_basic_features(ale,screen_state_atoms);
            num_basic_features = screen_state_atoms->size();
            if( (type_ > 1) && (screen_state_atoms != nullptr) ) {
                std::vector<int> basic_features(*screen_state_atoms);
                bpros_features_bitmap_ = std::vector<bool>(num_bpros_features_, false);
                compute_bpros_features(basic_features, *screen_state_atoms);
                num_bpros_features = screen_state_atoms->size() - num_basic_features;
                if( (type_ > 2) && (prev_screen_state_atoms != nullptr) ) {
                    bprot_features_bitmap_ = std::vector<bool>(num_bprot_features_, false);
                    compute_bprot_features(basic_features, *screen_state_atoms, *prev_screen_state_atoms);
                    num_bprot_features = screen_state_atoms->size() - num_basic_features - num_bpros_features;
                    if(type == 4){ 
                               // Compute adventure features
                            adventure_features_bitmap_ = std::vector<bool>(num_adventure_features_, false);
                            compute_adventure_features(*screen_state_atoms);
                            num_breakout_features = screen_state_atoms->size() - num_basic_features - num_bpros_features - num_bprot_features;

                                
                    }
                    else if(type == 5){ 
                        // Only compute your custom features here
                       breakout_features_bitmap_ = std::vector<bool>(num_breakout_features, false);
                       //compute_breakout_features(*screen_state_atoms);
                       //need to track the ball position @todo
                       num_breakout_features = screen_state_atoms->size()-num_bprot_features- num_basic_features - num_bpros_features;
                    }
                }
            }
        }

        logging::Logger::DebugMode(-100)
          << "screen:"
          << " #features=" << screen_state_atoms->size()
          << ", #basic=" << num_basic_features
          << ", #bpros=" << num_bpros_features
          << ", #bprot=" << num_bprot_features
          << std::endl;
    }

    void compute_basic_features(ALEInterface &ale,size_t c, size_t r, std::vector<int> *screen_state_atoms = 0) {
        assert((c < 16) && (r < 16));
        for( size_t ic = 0; ic < 10; ++ic ) {
            for( size_t ir = 0; ir < 15; ++ir ) {
                assert((15*r + ir < height_) && (10*c + ic < width_));
                pixel_t p = screen_.get(15*r + ir, 10*c + ic);
                pixel_t b = background_[(15*r + ir) * width_ + (10*c + ic)];

                // subtract/ammend background pixel
                if( p < b )
                    ammend_background_image(ale, 15*r + ir, 10*c + ic);
                else
                    p -= b;

                assert(p % 2 == 0); // per documentation, expecting 128 different colors!
                int pack = pack_basic_feature(this->height_ ,c, r, p >> 1);
                if( !basic_features_bitmap_[pack] ) {
                    basic_features_bitmap_[pack] = true;
                    if( screen_state_atoms != nullptr )
                        screen_state_atoms->push_back(pack);
                }
            }
        }
    }
    void compute_basic_features(ALEInterface &ale,std::vector<int> *screen_state_atoms = 0) {
        for( size_t c = 0; c <= width_ - 10; c += 10 ) { // 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150
            for( size_t r = 0; r <= height_ - 15; r += 15) { // 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195 210 225 240
                compute_basic_features(ale,c / 10, r / 15, screen_state_atoms);
            }
        }
    }

    void compute_bpros_features(const std::vector<int> &basic_features, std::vector<int> &screen_state_atoms) {
        std::pair<std::pair<size_t, size_t>, pixel_t> f1, f2;
        for( size_t j = 0; j < basic_features.size(); ++j ) {
            unpack_basic_feature(this->height_ ,basic_features[j], f1);
            for( size_t k = j; k < basic_features.size(); ++k ) {
                unpack_basic_feature(this->height_ , basic_features[k], f2);
                int pack = pack_bpros_feature(f1, f2);
                if( !bpros_features_bitmap_[pack - num_basic_features_] ) {
                    bpros_features_bitmap_[pack - num_basic_features_] = true;
                    screen_state_atoms.push_back(pack);
                }
            }
        }
    }

    void compute_bprot_features(const std::vector<int> &basic_features,
                                std::vector<int> &screen_state_atoms,
                                const std::vector<int> &prev_screen_state_atoms) {
        std::pair<std::pair<size_t, size_t>, pixel_t> f1, f2;
        for( size_t j = 0; j < basic_features.size(); ++j ) {
            unpack_basic_feature(this->height_ , basic_features[j], f1);
            for( size_t k = 0; k < prev_screen_state_atoms.size(); ++k ) {
                if( !is_basic_feature(this->height_, prev_screen_state_atoms[k]) ) break; // no more basic features in vector
                unpack_basic_feature(this->height_ ,prev_screen_state_atoms[k], f2);
                int pack = pack_bprot_feature(f1, f2);
                if( !bprot_features_bitmap_[pack - num_basic_features_ - num_bpros_features_] ) {
                    bprot_features_bitmap_[pack - num_basic_features_ - num_bpros_features_] = true;
                    screen_state_atoms.push_back(pack);
                }
            }
        }
    }

    static void compute_breakout_background(ALEInterface &ale, size_t num_frames) {
        // Reset to get intact bricks
        ale.reset_game();
        size_t width_ = ale.getScreen().width();
        size_t height_ = ale.getScreen().height();
        //fill_image(ale, initial_background_image_);
        ale.getScreenGrayscale(initial_background_image_ );
        /* std::cout << "Initial background image: " << std::endl;
        for(int i = 0; i < initial_background_image_.size(); i++){
            std::cout << static_cast<int>( initial_background_image_[i]) << " ";
            if(i % width_ == 0 && i != 0) std::cout << std::endl;
        }
        std::cout << std::endl; */
        
       
        num_initial_background_pixels_ = width_ * height_;      
    }
    
    // Add new helper functions
    void compute_adventure_features(std::vector<int> &screen_state_atoms) {
        const int base = num_basic_features_ + num_bpros_features_ + num_bprot_features_;
        
        // 1. Room detection
        const pixel_t room_color = screen_.get(5, 5);
        int room_id = -1;
        
        if (color_match(room_color, COLORS.at("yellow"))) {
            const pixel_t special_color = screen_.get(80, 80);
            room_id = color_match(special_color, COLORS.at("yellow")) ? 1 : 0;
        } else if (color_match(room_color, COLORS.at("green"))) room_id = 2;
        else if (color_match(room_color, COLORS.at("purple"))) room_id = 3;
        else if (color_match(room_color, COLORS.at("light_green"))) room_id = 5;
        else if (color_match(room_color, COLORS.at("blue"))) room_id = 6;
        else if (color_match(room_color, COLORS.at("black"))) room_id = 7;
        else if (color_match(room_color, COLORS.at("pink"))) room_id = 9;
        else if (color_match(room_color, COLORS.at("red"))) {
            // Special handling for red rooms
            if (Last_room_color == 3) room_id = 4;
            else if (Last_room_color == 7 || Last_room_color == 9) room_id = 8;
        }
        
        if (room_id >= 0) {
            //screen_state_atoms.push_back(base + room_id);
            int feature_index = pack_adventure_room_feature(room_id);
            if (feature_index < adventure_base_ + num_adventure_features_) {
                screen_state_atoms.push_back(feature_index);
                adventure_features_bitmap_[room_id] = true;
                Last_room_color = room_id;
            }
        }
        
        // 2. Cube detection
        auto cube_pos = find_cube_without_reference();
        if (cube_pos.first == -1) return;
        
        int cube_center_x = cube_pos.first + 2;
        int cube_center_y = cube_pos.second + 4;
        
        // 3. Item distance features
        add_item_distance(screen_state_atoms, adventure_base_+10, 0, cube_center_x, cube_center_y); // ykey
        add_item_distance(screen_state_atoms, adventure_base_+23, 1, cube_center_x, cube_center_y); // bkey
        add_item_distance(screen_state_atoms, adventure_base_+36, 2, cube_center_x, cube_center_y); // ysword
        add_item_distance(screen_state_atoms, adventure_base_+49, 3, cube_center_x, cube_center_y); // chalice
    }

    void add_item_distance(std::vector<int> &atoms, int base, int item_type, int cube_x, int cube_y) {
        int dist = get_item_distance(item_type, cube_x, cube_y);
        int bin = discretize_distance(dist);
        int feature_index = pack_adventure_item_feature(item_type, bin);
        
        // Validate feature index before pushing
        if (feature_index < adventure_base_ + num_adventure_features_) {
            atoms.push_back(feature_index);
            adventure_features_bitmap_[base - adventure_base_ + bin] = true;
        }
    }

    int discretize_distance(int dist) {
        if (dist < 0) return 0; // Unknown distance
        float d = static_cast<float>(dist) / 5.0f; // Convert to tens
        return static_cast<int>(d) +1; 
    }

    // Add color constants and helper functions
    const std::map<std::string, pixel_t> COLORS = {
        {"yellow", 193}, {"blue", 85}, {"red", 129}, {"black", 0},
        {"grey", 170}, {"green", 147}, {"purple", 157}, {"light_green", 157},
        {"pink", 107}, {"white", 255}
    };

    bool color_match(pixel_t c1, pixel_t c2) const {
        return std::abs(static_cast<int>(c1) - static_cast<int>(c2)) <= 5;
    }
    
    int get_item_distance(int item_type, int cube_x, int cube_y) const {
    
        auto items = detect_items_entire_screen(); // Assuming screen_ has a method to get items
        for( const auto& item : items) {
            const auto& [item_name, coords] = item;
            int x = coords.first;
            int y = coords.second;
            if((item_name == "yellow_key" && item_type == 0) ||(item_name == "black_key" && item_type == 1) 
            ||(item_name == "yellow_sword" && item_type == 2) ||(item_name == "chalice" && item_type == 3)){
                // Calculate Manhattan distance to the cube
            return std::abs(x - cube_x) + std::abs(y - cube_y);

            }
            
            

        }
     
        return -1;
    }

    std::pair<int, int> find_cube_without_reference() const {
        // Simplified cube detection
        const pixel_t cube_color = screen_.get(5, 5);
        const int search_margin = 20;
        
        for (int y = search_margin; y < height_ - search_margin; y++) {
            for (int x = search_margin; x < width_ - search_margin; x++) {
                if (color_match(screen_.get(y, x), cube_color) &&
                    color_match(screen_.get(y+1, x), cube_color) &&
                    color_match(screen_.get(y, x+1), cube_color) &&
                    color_match(screen_.get(y+1, x+1), cube_color)) {
                    return {x, y};
                }
            }
        }
        return {-1, -1};
    }
    // Detect items in the entire screen
    std::vector<std::pair<std::string, std::pair<int, int>>> detect_items_entire_screen() const {
        std::vector<std::pair<std::string, std::pair<int, int>>> detected_items;
        auto regions = regions_for_cube();
        auto cube_coords = find_cube_without_reference();

        // Collect candidate pixels (non-grey) in entire screen
        std::set<std::pair<int, int>> candidates;
            for (int y = 0; y < 210; y++) {
            for (int x = 0; x < 160; x++) {
                // Skip cube area if found
                if (cube_coords.first != -1 && 
                    x >= cube_coords.first && x < cube_coords.first + 4 &&
                    y >= cube_coords.second && y < cube_coords.second + 8) {
                    continue;
                }

                pixel_t px = screen_.get(y, x);
                if (!is_grey(px)) {
                    candidates.insert({x, y});
                }
            }
        }

        // Cluster candidate pixels
        auto clusters = cluster_pixels(candidates);

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
            pixel_t color = screen_.get(first_pixel.second, first_pixel.first);

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
    
    std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions_for_cube() const {
        std::vector<std::pair<std::pair<int,int>, std::pair<int, int>>> regions;

        // Get key colors from the screen
        pixel_t cube_color = screen_.get(5, 5);
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
            size_t idx = screen_.get(y, x);
            return color_match(screen_.get(y, x), COLORS.at(color));
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
    
    bool is_grey(pixel_t px) const {return color_match(px, COLORS.at(std::string("grey")));}
    
    std::vector<std::set<std::pair<int, int>>> cluster_pixels(const std::set<std::pair<int, int>>& candidates) const {
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
            pixel_t base_color = screen_.get(pixel.second, pixel.first);

            while (!queue.empty()) {
                auto [x, y] = queue.front();
                queue.pop();

                for (const auto& [dx, dy] : dirs) {
                    int nx = x + dx;
                    int ny = y + dy;
                    std::pair<int, int> neighbor = {nx, ny};
                    
                    // Check bounds and if already visited
                    if (nx < 0 || nx >= 160 || 
                        ny < 0 || ny >= 210) continue;
                    if (visited.find(neighbor) != visited.end()) continue;
                    
                    // Check color match and candidate status
                    pixel_t neighbor_color = screen_.get(ny, nx);
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

    // features
    typedef std::pair<size_t, size_t> patch_t;
    typedef std::pair<patch_t, pixel_t> basic_feature_t;
    typedef std::pair<size_t, size_t> offset_t;
    typedef std::pair<offset_t, std::pair<pixel_t, pixel_t> > bpros_feature_t;
    typedef std::pair<offset_t, std::pair<pixel_t, pixel_t> > bprot_feature_t;

    // basic features
    static bool is_basic_feature(size_t height_, int pack) {
       
        int num = int(num_basic_features_);
        if(height_ == 250) num = 16*16*128 ;
        //std:: cout << "height: " << height_ << std::endl;
        //std:: cout << "pack: " << pack << " <" << num << std::endl;
        return (pack >= 0) && (pack < num );
    }
    static int pack_basic_feature( size_t height_, size_t c, size_t r, pixel_t p) {
        //size_t height_ = ale.getScreen().height();
        size_t limit = 14; 
        if (height_ == 250) limit = 16; 
        assert((c < 16) && (r < limit));
        assert((p >= 0) && (p < 128));
        int pack = ((limit * c + r) << 7) + p;
        assert(is_basic_feature(height_,pack));
        return pack;
    }
    static int pack_basic_feature(size_t height_, const basic_feature_t &bf) {
        return pack_basic_feature(height_, bf.first.first, bf.first.second, bf.second);
    }
    static void unpack_basic_feature(size_t height_, int pack, basic_feature_t &bf) {
        assert(is_basic_feature(height_,pack));
        size_t limit = 14; 
        if (height_ == 250) limit = 16; 
        bf.first.first = (pack >> 7) / limit;
        bf.first.second = (pack >> 7) % limit;
        bf.second = pack & 127;
        assert(pack == pack_basic_feature(height_,bf));
    }

    // B-PROS features
    static bool is_bpros_feature(int pack) {
        return (pack >= int(num_basic_features_)) && (pack < int(num_basic_features_ + num_bpros_features_));
    }
    static int pack_bpros_feature(int dc, int dr, pixel_t p1, pixel_t p2) {
        assert((-15 <= dc) && (dc <= 15));
        assert((-13 <= dr) && (dr <= 13));
        assert((p1 >= 0) && (p1 < 128));
        assert((p2 >= 0) && (p2 < 128));
        assert(p1 <= p2);
        int pack = 0;
        if( p1 < p2 ) {
            pack = ((15 + dc) * 27 + (13 + dr)) * 128 * 127 / 2;
            pack += p1 * 127 - p1 * (1 + p1) / 2 + p2 - 1;
            assert((pack >= 0) && (pack < int(num_bpros_features_t0_)));
        } else if( (dc != 0) || (dr != 0) ) {
            assert(p1 == p2);
            if( (dc < 0) || ((dc == 0) && (dr < 0)) ) {
                dc = -dc;
                dr = -dr;
            }
            assert((dc > 0) || ((dc == 0) && (dr > 0)));

            if( dc > 0 ) {
                pack = ((dc - 1) * 27 + (13 + dr)) * 128 + p1;
                assert((pack >= 0) && (pack < 15 * 27 * 128));
            } else {
                assert(dr > 0);
                pack = 15 * 27 * 128 + (dr - 1) * 128 + p1;
            }

            assert((pack >= 0) && (pack < int(num_bpros_features_t1_)));
            pack += num_bpros_features_t0_;
        } else {
            assert((p1 == p2) && (dc == 0) && (dr == 0));
            pack = p1;
            assert((pack >= 0) && (pack < int(num_bpros_features_t2_)));
            pack += num_bpros_features_t0_ + num_bpros_features_t1_;
        }
        pack += num_basic_features_;
        assert(is_bpros_feature(pack));
        return pack;
    }
    static int pack_bpros_feature(const bpros_feature_t &cf) {
        return pack_bpros_feature(cf.first.first, cf.first.second, cf.second.first, cf.second.second);
    }
    static int pack_bpros_feature(const basic_feature_t &bf1, const basic_feature_t &bf2) {
        int dc = bf1.first.first - bf2.first.first;
        int dr = bf1.first.second - bf2.first.second;
        if( bf1.second <= bf2.second )
            return pack_bpros_feature(dc, dr, bf1.second, bf2.second);
        else
            return pack_bpros_feature(-dc, -dr, bf2.second, bf1.second);
    }
    static void unpack_bpros_feature(int pack, bpros_feature_t &cf) {
        assert(0);
    }

    // B-PROT features
    static bool is_bprot_feature(int pack) {
        return (pack >= int(num_basic_features_ + num_bpros_features_)) && (pack < int(num_basic_features_ + num_bpros_features_ + num_bprot_features_));
    }
    static int pack_bprot_feature(int dc, int dr, pixel_t p1, pixel_t p2) {
        assert((-15 <= dc) && (dc <= 15));
        assert((-13 <= dr) && (dr <= 13));
        assert((((15 + dc) * 27 + (13 + dr)) * 128 + p1) * 128 + p2 < int(num_bprot_features_));
        return num_basic_features_ + num_bpros_features_ + (((15 + dc) * 27 + (13 + dr)) * 128 + p1) * 128 + p2;
    }
    static int pack_bprot_feature(const bprot_feature_t &cf) {
        return pack_bprot_feature(cf.first.first, cf.first.second, cf.second.first, cf.second.second);
    }
    static int pack_bprot_feature(const basic_feature_t &bf1, const basic_feature_t &bf2) {
        int dc = bf1.first.first - bf2.first.first;
        int dr = bf1.first.second - bf2.first.second;
        return pack_bprot_feature(dc, dr, bf1.second, bf2.second);
    }
    static void unpack_bprot_feature(int pack, bprot_feature_t &cf) {
        assert(pack >= 0);
        pixel_t p2 = pack % 128;
        pixel_t p1 = (pack >> 7) % 128;
        int dr = ((pack >> 14) % 27) - 13;
        assert((pack >> 14) / 27 < 31);
        int dc = ((pack >> 14) / 27) - 15;
        cf = std::make_pair(std::make_pair(dc, dr), std::make_pair(p1, p2));
        assert(pack == pack_bprot_feature(cf));
    }

    //adventure features
    // Add validation function
    static bool is_adventure_feature(int pack) {
        return (pack >= int(adventure_base_)) &&  (pack < int(adventure_base_ + num_adventure_features_));
    }
        // Add packing functions for adventure features
    static int pack_adventure_room_feature(int room_id) {
        assert(room_id >= 0 && room_id < 10);
        return adventure_room_start_ + room_id;
    }

    static int pack_adventure_item_feature(int item_type, int distance_bin) {
        assert(item_type >= 0 && item_type < 4);
        assert(distance_bin >= 0 && distance_bin < 75);
        
        switch(item_type) {
            case 0: return adventure_ykey_start_ + distance_bin;
            case 1: return adventure_bkey_start_ + distance_bin;
            case 2: return adventure_ysword_start_ + distance_bin;
            case 3: return adventure_chalice_start_ + distance_bin;
            default: return -1;
        }
    }
};

#endif

