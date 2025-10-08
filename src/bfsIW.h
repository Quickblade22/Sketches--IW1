// (c) 2017 Blai Bonet

#ifndef BFS_IW_H
#define BFS_IW_H

#include <cassert>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "sim_planner.h"
#include "logger.h"

struct BfsIW : SimPlanner {
    const int screen_features_;
    const float time_budget_;
    const bool novelty_subtables_;
    const bool random_actions_;
    const size_t max_rep_;
    const float discount_;
    const float alpha_;
    const bool use_alpha_to_update_reward_for_death_;
    const int nodes_threshold_;
    const bool break_ties_using_rewards_;
    const int depth_to_search; 
    mutable size_t num_expansions_;
    mutable float total_time_;
    mutable float expand_time_;
    mutable size_t root_height_;
    mutable bool random_decision_;
    int game; 
    mutable std::deque<Action> fulfillment_branch_;
    mutable  Node *best_node; 
    mutable bool print = false; 
    mutable bool gdragon_print_screen = false; 
    BfsIW(ALEInterface &sim,
          size_t frameskip,
          bool use_minimal_action_set,
          size_t num_tracked_atoms,
          int screen_features,
          float simulator_budget,
          float time_budget,
          bool novelty_subtables,
          bool random_actions,
          size_t max_rep,
          float discount,
          float alpha,
          bool use_alpha_to_update_reward_for_death,
          int nodes_threshold,
          bool break_ties_using_rewards,int games ,int look_ahead = 100, bool printing_sketches = false)
      : SimPlanner(sim, frameskip, use_minimal_action_set, simulator_budget, num_tracked_atoms, look_ahead, printing_sketches),
        screen_features_(screen_features),
        time_budget_(time_budget),
        novelty_subtables_(novelty_subtables),
        random_actions_(random_actions),
        max_rep_(max_rep),
        discount_(discount),
        alpha_(alpha),
        use_alpha_to_update_reward_for_death_(use_alpha_to_update_reward_for_death),
        nodes_threshold_(nodes_threshold),
        break_ties_using_rewards_(break_ties_using_rewards), game(games) ,depth_to_search(look_ahead) {
           initalize_sketches_adventure();
           //initialize_sketches_seaquest();
           /* if(game == 0) initialize_sketches();
            else if (game == 1) initialize_sketches_breakout(); 
            else */
           
    }
    virtual ~BfsIW() { }
    mutable int action_nr = 0;
    virtual std::string name() const {
        return std::string("bfs(")
          + "frameskip=" + std::to_string(frameskip_)
          + ",minimal-action-set=" + std::to_string(use_minimal_action_set_)
          + ",features=" + std::to_string(screen_features_)
          + ",simulator-budget=" + std::to_string(simulator_budget_)
          + ",time-budget=" + std::to_string(time_budget_)
          + ",novelty-subtables=" + std::to_string(novelty_subtables_)
          + ",random-actions=" + std::to_string(random_actions_)
          + ",max-rep=" + std::to_string(max_rep_)
          + ",discount=" + std::to_string(discount_)
          + ",alpha=" + std::to_string(alpha_)
          + ",use-alpha-to-update-reward-for-death=" + std::to_string(use_alpha_to_update_reward_for_death_)
          + ",nodes-threshold=" + std::to_string(nodes_threshold_)
          + ",break-ties-using-rewards=" + std::to_string(break_ties_using_rewards_)
          + ")";
    }

    virtual bool random_decision() const {
        return random_decision_;
    }
    virtual size_t height() const {
        return root_height_;
    }
    virtual size_t expanded() const {
        return num_expansions_;
    }
    const bool printing_debug = false; // Set to true to enable debug printing
    const bool transition_printing_debug = true; // Set to true to enable transition debug printing
    //const bool transition_printing_debugs = true; // Set to true to enable transition debug printing
    virtual Node* get_branch(ALEInterface &env,
                             const std::vector<Action> &prefix,
                             Node *root,
                             float last_reward,
                             std::deque<Action> &branch) const {
        assert(!prefix.empty());

        logging::Logger::Info << "**** bfs: get branch ****" << std::endl;
        logging::Logger::Info << "prefix: sz=" << prefix.size() << ", actions=";
        print_prefix(logging::Logger::Info, prefix);
        logging::Logger::Continuation(logging::Logger::Info) << std::endl;
        logging::Logger::Info << "input:"
                     << " #nodes=" << (root == nullptr ? "na" : std::to_string(root->num_nodes()))
                     << ", #tips=" << (root == nullptr ? "na" : std::to_string(root->num_tip_nodes()))
                     << ", height=" << (root == nullptr ? "na" : std::to_string(root->height_))
                     << std::endl;

        // reset stats and start timer
        reset_stats();
        float start_time = Utils::read_time_in_seconds();
        float debug_time = 0.0f;
        float debug_time_stop = 0.0f;                        
        // novelty table
        std::map<int, std::vector<int> > novelty_table_map;

        // construct root node
        assert((root == nullptr) || (root->action_ == prefix.back()));
        if( root == nullptr ) {
            Node *root_parent = new Node(nullptr, PLAYER_A_NOOP, -1);
            root_parent->state_ = new ALEState;
            apply_prefix(sim_, initial_sim_state_, prefix, root_parent->state_);
            root = new Node(root_parent, prefix.back(), 0);
        }
        assert(root->parent_ != nullptr);
        root->grandfather = root->parent_->parent_;
        root->parent_->parent_ = nullptr;

        // if root has some children, make sure it has all children
        if( root->num_children_ > 0 ) {
            assert(root->first_child_ != nullptr);
            std::set<Action> root_actions;
            for( Node *child = root->first_child_; child != nullptr; child = child->sibling_ )
                root_actions.insert(child->action_);

            // complete children
            assert(root->num_children_ <= int(action_set_.size()));
            if( root->num_children_ < int(action_set_.size()) ) {
                for( size_t k = 0; k < action_set_.size(); ++k ) {
                    if( root_actions.find(action_set_[k]) == root_actions.end() )
                        root->expand(action_set_[k]);
                }
            }
            assert(root->num_children_ == int(action_set_.size()));
        } else {
            // make sure this root node isn't marked as frame rep
            root->parent_->feature_atoms_.clear();
        }

        // normalize depths and recompute path rewards
        root->parent_->depth_ = -1;
        root->normalize_depth();
        root->reset_frame_rep_counters(frameskip_);
        root->recompute_path_rewards(root);

        // construct/extend lookahead tree
        if( int(root->num_nodes()) < nodes_threshold_ ) {
            bfs(prefix, root, novelty_table_map);
        }

        // if nothing was expanded, return random actions (it can only happen with small time budget)
        if( root->num_children_ == 0 ) {
             logging::Logger::Info << logging::Logger::green() << " very small time bugdet" ; 
            assert(root->first_child_ == nullptr);
            assert(time_budget_ != std::numeric_limits<float>::infinity());
            random_decision_ = true;
            branch.push_back(random_action());
        } else {
            assert(root->first_child_ != nullptr);

            // backup values and calculate heights
            root->backup_values(discount_);
            root->calculate_height();
            root_height_ = root->height_;

            // print info about root node
            logging::Logger::Debug << logging::Logger::green()
                          << "root:"
                          << " value=" << root->value_
                          << ", imm-reward=" << root->reward_
                          << ", children=[";
            if(transition_printing_debug) std::cout << "Root node preconditions: " << std::endl;
             if(root->parent_->screen_pixels_.size() > 0)  {
                bool printing = transition_printing_debug ? true : false;
                root->pre = check_sketches_preconditions(root->parent_->screen_pixels_,root->screen_pixels_, *this, root, printing, true);
             }
            else {
                bool printing = transition_printing_debug ? true : false;
                root->pre = check_sketches_preconditions(root->screen_pixels_,root->screen_pixels_, *this, root, printing, true);
            }
            if(transition_printing_debug) std::cout << "---------------------------------------------------------------------------------" << std::endl;
            
            
            // compute branch

            int pres = 0; 
            for(auto i: root->pre){
                pres += static_cast<int>(i); 
            }
           /*if(root->pre[5] && !print ) {
                std::cout << " reaching gdragon active " << std::endl; 
                printing_screen(root->screen_pixels_);
                print = true;
            }else if(print && !root->pre[5]) {
                std::cout << " reaching gdragon inactive " << std::endl; 
                printing_screen(root->screen_pixels_);
            }
            if(root->pre[6] && !gdragon_print_screen ) {
                std::cout << " Killing gdragon active " << std::endl; 
                //printing_screen(root->screen_pixels_);
                gdragon_print_screen = true;
            }else if(gdragon_print_screen && !root->pre[6]) {
                std::cout << " Killing gdragon inactive " << std::endl; 
                printing_screen(root->screen_pixels_);
            }*/
           
            if(printing_debug)  std::cout<< "starting branch computation with preconditions: " << pres << std::endl;
            if( pres != 0 ) {
                if(transition_printing_debug)   std::cout << "sketches exploitation with depth lookahead" << std::endl;
                // Use depth-based sketch evaluation (e.g., look 3 steps ahead)
                 //trial using fulfillment branch
                if(printing_debug) std::cout << "fulfillment branch size: " << fulfillment_branch_.size() << std::endl;
                
                if(!fulfillment_branch_.empty()) {
                     branch.insert(branch.end(), fulfillment_branch_.begin(), fulfillment_branch_.end());
                     Node* temp_node_parent = (best_node->parent_ != nullptr && !best_node->parent_->screen_pixels_.empty()) ? best_node->parent_ : best_node;
                     if(transition_printing_debug) {
                            debug_time = Utils::read_time_in_seconds();
                            std::cout << "Action in fulfillment branch: "; 
                            for(const auto& act:fulfillment_branch_) std::cout << act << " " ;
                            std::cout  << std::endl;
                            bool chalicer_bool = chalicer(best_node->screen_pixels_, temp_node_parent->screen_pixels_);
                            std::cout<< "At fulfillment branch end, chalicer is " << chalicer_bool << std::endl;
                            if(chalicer_bool) {
                                std::cout << "Best node is chalicer_bool, printing screen from current node until root node/node where chalicer_bool true " << std::endl;
                                printing_screen(best_node->screen_pixels_);
                                auto items =  detect_items_entire_screen(best_node->screen_pixels_, temp_node_parent->screen_pixels_);
                                auto temp2 = highlight_cube(best_node->screen_pixels_, best_node->screen_pixels_);
                                for(const auto& item: items) {
                                    if(item.first == "chalice") {
                                        std::cout << "chalice at " << item.second.first << ", " << item.second.second << " and cube is at " <<  temp2.first << ", " << temp2.second << std::endl;
                                        std::cout << "chalice cluster has the color " << static_cast<int>(best_node->screen_pixels_[item.second.second * SCREEN_WIDTH + item.second.first]) << std::endl;
                                    }
                                }
                                auto temp_node = best_node->parent_;
                                int i = 1; 
                                while(temp_node != nullptr && temp_node != root ) {
                                    std::cout << "best node's "<<  i <<" father and its  action: " << temp_node->action_ << " and chalicer is " << temp_node->node_chalicet  << std::endl;
                                    bool temp_bool = chalicer(temp_node->screen_pixels_,temp_node_parent->screen_pixels_);
                                    if(temp_bool) {
                                        std::cout << "Found chalicer at depth " << i << std::endl;
                                        auto items =  detect_items_entire_screen(temp_node->screen_pixels_, temp_node_parent->screen_pixels_);
                                        auto temp2 = highlight_cube(temp_node->screen_pixels_, temp_node->screen_pixels_);
                                        for(const auto& item: items) {
                                            if(item.first == "chalice") {
                                                std::cout << "chalice at " << item.second.first << ", " << item.second.second << " and cube is at " <<  temp2.first << ", " << temp2.second << std::endl;
                                            }
                                        }
                                        printing_screen(temp_node->screen_pixels_);
                                    }
                                    temp_node = temp_node->parent_;
                                    ++i;
                                }
                            }
                           /* bool temp_before =  best_node->node_chalicet;
                            if(temp_before && transition_printing_debug) {
                                std::cout << "father_node is " << 
                                best_node->parent_->action_ << " and bkeyt" <<  best_node->parent_->node_chalicet 
                                << " current node is " << best_node->action_  << "and bkeyt " 
                                << best_node->node_chalicet << std::endl;
                            }*/
                            if(best_node->node_chalicet){
                                std::cout << "Best node is chalicet, printing screen from current node until root node/node where chalicet true " << std::endl;
                                auto temp_node = best_node->parent_;
                                int i = 1; 
                                while(temp_node != nullptr && temp_node != root ) {
                                    std::cout << "best node's "<<  i <<" father and its  action: " << temp_node->action_ << " and chalicet is " << temp_node->node_chalicet  << std::endl;
                                    if(temp_node->node_chalicet) {
                                        auto temp2 = highlight_cube(temp_node->screen_pixels_, temp_node->screen_pixels_);
                                        std::cout << "Found chalicet at depth " << i << " and cube is at " <<  temp2.first << ", " << temp2.second << " and chalicet at "  << std::endl;

                                        printing_screen(temp_node->screen_pixels_);
                                    }
                                    temp_node = temp_node->parent_;
                                    ++i;
                                }
                            }
                            //variables of best_node
                            
                            bool temp_ykeyt = best_node->node_ykeyt; 
                            bool temp_bkeyt = best_node->node_bkeyt;
                            bool temp_yswrt = best_node->node_yswrt;
                            bool temp_chalicet = best_node->node_chalicet;
                            bool temp_ydragon = best_node->node_ydragon;
                            bool temp_gdragon = best_node->node_gdragon;
                            int last_room_color = best_node->node_Last_room_color;
                            //setting the right states 
                            set_item_state(best_node, true);
                        
                            std::vector<bool> temp_goal = {false};
                            std::vector<bool> temp_pre = {false};
                            std::cout<< "checking sketches preconditions and goals for best node: " << best_node->action_ << std::endl;
                            std::cout << "ykeyt: " << best_node->node_ykeyt <<  " parents_ykeyt: " << best_node->parent_->node_ykeyt
                                      << " yswrt: " << best_node->node_yswrt <<  " parents_yswrt: " << best_node->parent_->node_yswrt
                                      <<  " ydragon" << best_node->node_ydragon << " parents: " <<  best_node->parent_->node_ydragon
                                      <<  " gdragon" << best_node->node_gdragon << " parents: " <<  best_node->parent_->node_gdragon
                                      << " last_room_color: " << best_node->node_Last_room_color    << std::endl;
                            bool trial_debug = true;
                            bool recompute = true; 
                            if(recompute){
                                if(best_node->parent_ != nullptr ){
                                    if(best_node->parent_->parent_ != nullptr) temp_goal = check_sketches_goals(best_node->parent_->screen_pixels_, best_node->screen_pixels_, best_node->parent_->screen_pixels_, *this, best_node,trial_debug);
                                    else  temp_goal = check_sketches_goals(best_node->parent_->screen_pixels_, best_node->screen_pixels_, best_node->screen_pixels_, *this, best_node,trial_debug); 
                                    temp_pre = check_sketches_preconditions(best_node->parent_->screen_pixels_, best_node->screen_pixels_, *this, best_node,trial_debug);     
                                }else {
                                    temp_goal = check_sketches_goals(best_node->screen_pixels_, best_node->screen_pixels_, best_node->screen_pixels_, *this, best_node,trial_debug);
                                    temp_pre = check_sketches_preconditions(best_node->screen_pixels_, best_node->screen_pixels_, *this, best_node,trial_debug);
                                
                                }
                            }
                          
                            std::cout << "----------------------------------------------------- " << std::endl;
                            //reseting best_node variables
                            if(best_node->node_ykeyt != temp_ykeyt || best_node->node_bkeyt != temp_bkeyt || best_node->node_yswrt != temp_yswrt || best_node->node_chalicet != temp_chalicet || best_node->node_ydragon != temp_ydragon || best_node->node_gdragon != temp_gdragon || best_node->node_Last_room_color != last_room_color) {
                                std::cout << "touching variables changed" << std::endl;
                            }
                            best_node->node_ykeyt = temp_ykeyt;
                            best_node->node_bkeyt = temp_bkeyt;
                            best_node->node_yswrt = temp_yswrt;
                            best_node->node_chalicet = temp_chalicet;
                            best_node->node_ydragon = temp_ydragon;
                            best_node->node_gdragon = temp_gdragon;
                            best_node->node_Last_room_color = last_room_color;
                            
                        }
                        debug_time_stop = Utils::read_time_in_seconds() - debug_time;
                }else{
                    std::cout << "fulfillment branch is empty, falling back" << std::endl;
                    root->best_sketch_branch(branch, root->pre, depth_to_search, discount_, priority_,action_nr);

                }
                /*
                else {
                    //const int lookahead_depth = 10; // Adjust this value as needed
                    root->best_sketch_branch(branch, root->pre, depth_to_search, discount_, priority_,action_nr); 
                    action_nr++;
                
                }  
                // Fallback if no branch was found (shouldn't happen)
                
                if (branch.empty()) {
                    if(transition_printing_debug)  logging::Logger::Info << logging::Logger::green() << "fallback to original behavior " << std::endl;
                    root->best_branch(branch, discount_);
                }
                */
            } 
            else {
                std::cout << "random actions" << std::endl;
                if( random_actions_ ) {
                    random_decision_ = true;
                    branch.push_back(random_zero_value_action(root, discount_));
                } else {
                    root->longest_zero_value_branch(discount_, branch);
                    assert(!branch.empty());
                }
            }
            //clearing fulfillment branch
            fulfillment_branch_.clear();
            if(transition_printing_debug){
                  Node* temp_node_parent = (best_node->parent_ != nullptr && !best_node->parent_->screen_pixels_.empty()) ? best_node->parent_ : best_node;
                  Node* root_parent = (root->parent_ != nullptr && !root->parent_->screen_pixels_.empty()) ? root->parent_ : root;
                  std::cout << "distance to items: ykey: best node = " << ykey_dist(best_node->screen_pixels_, temp_node_parent->screen_pixels_)  << " root= " << ykey_dist(root->screen_pixels_, root_parent->screen_pixels_)
                                      << " ysword: best node = " << ysword_dist(best_node->screen_pixels_, temp_node_parent->screen_pixels_) << " root= " << ysword_dist(root->screen_pixels_, root_parent->screen_pixels_)
                                      << " bkey: best node = " << bkey_dist(best_node->screen_pixels_, temp_node_parent->screen_pixels_) << " root= " << bkey_dist(root->screen_pixels_, root_parent->screen_pixels_)
                                      << " chalice: best node = " << chalice_dist(best_node->screen_pixels_, temp_node_parent->screen_pixels_) << " root= " << chalice_dist(root->screen_pixels_, root_parent->screen_pixels_)
                                      << std::endl;
            }
            // make sure states along branch exist (only needed when doing partial caching)
            generate_states_along_branch(root, branch, screen_features_, alpha_, use_alpha_to_update_reward_for_death_);

            // print branch
            assert(!branch.empty());
            logging::Logger::Debug << "branch:"
                          << " value=" << root->value_
                          << ", size=" << branch.size()
                          << ", actions:"
                          << std::endl;
            //root->print_branch(logos_, branch);
        }
        
        // stop timer and print stats
        total_time_ = Utils::read_time_in_seconds() - start_time + debug_time_stop;
        print_stats(logging::Logger::Stats, *root, novelty_table_map);
        if(impotant_debug) std::cout << "bfs: total time: " << total_time_ << " seconds" << std::endl;
        // return root node
        return root;
    }

    // breadth-first search with ties broken in favor of bigger path reward
    struct NodeComparator {
        bool break_ties_using_rewards_;
        NodeComparator(bool break_ties_using_rewards) : break_ties_using_rewards_(break_ties_using_rewards) {
        }
        bool operator()(const Node *lhs, const Node *rhs) const {
            return
              (lhs->depth_ > rhs->depth_) ||
              (break_ties_using_rewards_ && (lhs->depth_ == rhs->depth_) && (lhs->path_reward_ < rhs->path_reward_));
        }
    };

    void bfs(const std::vector<Action> &prefix, Node *root, std::map<int, std::vector<int> > &novelty_table_map) const {
        // priority queue
        if(transition_printing_debug) std::cout << "simulator calls: " << int(simulator_calls_) << " simulator budget: " << simulator_budget_ << std::endl;
        NodeComparator cmp(break_ties_using_rewards_);
        std::priority_queue<Node*, std::vector<Node*>, NodeComparator> q(cmp);

        // add tip nodes to queue
        add_tip_nodes_to_queue(root, q);
        logging::Logger::Info << "queue: sz=" << q.size() << std::endl;

        // explore in breadth-first manner
        int count = 0; 
        float start_time = Utils::read_time_in_seconds();
        bool queue_empty = q.empty();
        bool simulator_budget_reached = (int(simulator_calls_) < simulator_budget_);
        bool time_budget_reached = (Utils::read_time_in_seconds() - start_time < time_budget_);
        bool stop = !queue_empty && simulator_budget_reached && time_budget_reached && fulfillment_branch_.empty();
        while( stop ) {
            Node *node = q.top();
            q.pop();
            
            // print debug info
            logging::Logger::Continuation(logging::Logger::Debug) << node->depth_ << "@" << node->path_reward_ << std::flush;

            // update node info
            assert((node->num_children_ == 0) && (node->first_child_ == nullptr));
            assert(node->visited_ || (node->is_info_valid_ != 2));
            if( node->is_info_valid_ != 2 ) {
                update_info(node, screen_features_, alpha_, use_alpha_to_update_reward_for_death_);
                assert((node->num_children_ == 0) && (node->first_child_ == nullptr));
                node->visited_ = true;
                
            }
            if(node->parent_ != nullptr && !node->parent_->screen_pixels_.empty()) {
                    node->pre = check_sketches_preconditions(node->parent_->screen_pixels_, node->screen_pixels_, *this, node);
                    std::vector<pixel_t> typical = node->parent_->screen_pixels_;
                    if(node->grandfather != nullptr && !node->grandfather->screen_pixels_.empty()) {
                        typical = node->grandfather->screen_pixels_;
                    }
                    node->post = check_sketches_goals(node->parent_->screen_pixels_, node->screen_pixels_, typical, *this, node);
            } else {
                    node->pre = check_sketches_preconditions(node->screen_pixels_, node->screen_pixels_, *this, node);
                    node->post = check_sketches_goals(node->screen_pixels_, node->screen_pixels_, node->screen_pixels_, *this, node);
            }
            //trial setting the fulfillment branch

            if( printing_debug && !fulfillment_branch_.empty()  && count == 0) {
                std::cout << "fulfillment not empty" << std::endl;
                count ++;
            }
            
            // break out of the loop if node found
            if (fulfillment_branch_.empty()) {

                for (size_t i = 0; i < node->post.size(); ++i) {
                    if( i == 5 && root->pre[5] ) printing_sketches_debug = true;
                    if (root->pre[i] && node->post[i]) {
                        // Reconstruct branch from root to this node
                        std::deque<Action> temp_branch;
                        Node* temp = node;
                        best_node = node;
                        while (temp != root) {
                            temp_branch.push_front(temp->action_);
                            temp = temp->parent_;
                        }
                        fulfillment_branch_ = temp_branch;
                        
                        break;
                    }
                }
            }
            
            // check termination at this node
            if( node->terminal_ ) {
                logging::Logger::Continuation(logging::Logger::Debug) << "t" << "," << std::flush;
                continue;
            }

            // verify max repetitions of feature atoms (screen mode)
            if( node->frame_rep_ > int(max_rep_) ) {
                logging::Logger::Continuation(logging::Logger::Debug) << "r" << node->frame_rep_ << "," << std::flush;
                continue;
            }

            // calculate novelty and prune
            if( node->frame_rep_ == 0 ) {
                // calculate novelty
                std::vector<int> &novelty_table = get_novelty_table(node, novelty_table_map, novelty_subtables_);
                int atom = get_novel_atom(node->depth_, node->feature_atoms_, novelty_table);
                assert((atom >= 0) && (atom < int(novelty_table.size())));

                // prune node using novelty
                if( novelty_table[atom] <= node->depth_ ) {
                    //std::cout<< "pruning node with atom " << atom << " at depth " << node->depth_ << std::endl;
                    logging::Logger::Continuation(logging::Logger::Debug) << "p" << "," << std::flush;
                    continue;
                }

                // update novelty table
                update_novelty_table(node->depth_, node->feature_atoms_, novelty_table);
            }
            logging::Logger::Continuation(logging::Logger::Debug) << "+" << std::flush;

            // expand node
            if( node->frame_rep_ == 0 ) {
                ++num_expansions_;
                float start_time = Utils::read_time_in_seconds();
                node->expand(action_set_, false);
                expand_time_ += Utils::read_time_in_seconds() - start_time;
            } else {
                assert((node->parent_ != nullptr) && (screen_features_ > 0));
                node->expand(node->action_);
            }
            assert((node->num_children_ > 0) && (node->first_child_ != nullptr));
            
            logging::Logger::Continuation(logging::Logger::Debug) << node->num_children_ << "," << std::flush;

            // add children to queue
            for( Node *child = node->first_child_; child != nullptr; child = child->sibling_ ) q.push(child);
            
            simulator_budget_reached = (int(simulator_calls_) < simulator_budget_);
            time_budget_reached = (Utils::read_time_in_seconds() - start_time < time_budget_);
            stop = !queue_empty && simulator_budget_reached && time_budget_reached && fulfillment_branch_.empty();
            if(printing_debug){
               std::cout<< "bfs queue ongonging" << std::endl;
            }
            if(!stop ) {
                logging::Logger::Info << "bfs: stopping search, queue empty=" << queue_empty
                                      << ", simulator budget reached=" << simulator_budget_reached
                                      << ", time budget reached=" << time_budget_reached
                                      << ", fulfillment branch full=" << !fulfillment_branch_.empty() << std::endl;
            }
            
        }
        logging::Logger::Continuation(logging::Logger::Debug) << std::endl;
    }

    void add_tip_nodes_to_queue(Node *node, std::priority_queue<Node*, std::vector<Node*>, NodeComparator> &pq) const {
        std::deque<Node*> q;
        q.push_back(node);
        while( !q.empty() ) {
            Node *n = q.front();
            q.pop_front();
            if( n->num_children_ == 0 ) {
                assert(n->first_child_ == nullptr);
                pq.push(n);
            } else {
                assert(n->first_child_ != nullptr);
                for( Node *child = n->first_child_; child != nullptr; child = child->sibling_ )
                    q.push_back(child);
            }
        }
    }

    void reset_stats() const {
        SimPlanner::reset_stats();
        num_expansions_ = 0;
        total_time_ = 0;
        expand_time_ = 0;
        root_height_ = 0;
        random_decision_ = false;
        fulfillment_branch_.clear();
    }

    void print_stats(logging::Logger::mode_t logger_mode, const Node &root, const std::map<int, std::vector<int> > &novelty_table_map) const {
        logger_mode << "decision-stats:"
                    << " #entries=[";

        for( std::map<int, std::vector<int> >::const_iterator it = novelty_table_map.begin(); it != novelty_table_map.end(); ++it )
            logging::Logger::Continuation(logger_mode) << it->first << ":" << num_entries(it->second) << "/" << it->second.size() << ",";

        logging::Logger::Continuation(logger_mode)
          << "]"
          << " #nodes=" << root.num_nodes()
          << " #tips=" << root.num_tip_nodes()
          << " height=[" << root.height_ << ":";

        for( Node *child = root.first_child_; child != nullptr; child = child->sibling_ )
            logging::Logger::Continuation(logger_mode) << child->height_ << ",";

        logging::Logger::Continuation(logger_mode)
          << "]"
          << " #expansions=" << num_expansions_
          << " #sim=" << simulator_calls_
          << " total-time=" << total_time_
          << " simulator-time=" << sim_time_
          << " reset-time=" << sim_reset_time_
          << " get/set-state-time=" << sim_get_set_state_time_
          << " expand-time=" << expand_time_
          << " update-novelty-time=" << update_novelty_time_
          << " get-atoms-calls=" << get_atoms_calls_
          << " get-atoms-time=" << get_atoms_time_
          << " novel-atom-time=" << novel_atom_time_
          << std::endl;
    }
};

#endif

