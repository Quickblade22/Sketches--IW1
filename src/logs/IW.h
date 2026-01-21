// 2025 Aaditya Mehta
#ifndef ITERATED_WIDTH_H
#define ITERATED_WIDTH_H

#include <queue>
#include <set>
#include <vector>
#include <algorithm>
#include <string>
#include <map>

#include "sim_planner.h"
#include "logger.h"

// Iterated Width (IW) Planner - Implements IW(k) algorithm with novelty pruning
class IteratedWidth : public SimPlanner {
    // Configuration parameters
    const bool novelty_subtable; // Flag for novelty subtable usage (moved up)
    const int width_;          // IW parameter (k-value for tuple size)
    const int screen_features_; // Feature extraction mode (0=RAM, 1+=Screen)
    const float time_budget_;   // Max time allowed for planning
    const bool use_minimal_action_set_;
    const size_t max_rep_;      // Max frame repetitions allowed
    const float discount_;      // Discount factor for value backup
    const float alpha_;         // Reward scaling factor
    const bool use_alpha_to_update_reward_for_death_;
    const int nodes_threshold_; // Node count limit for tree construction
    const size_t max_depth_;    // Max search depth
    
    // Runtime statistics
    mutable size_t num_expansions_;
    mutable float total_time_;
    mutable float expand_time_;
    mutable std::set<std::vector<int>> novelty_table_; // Stores seen feature tuples
    mutable std::map<int, std::set<std::vector<int>>> novelty_table_map_;  // For subtables
    mutable size_t root_height_;      // Height of the search tree
    mutable bool random_decision_;   // Flag for random action fallback


public:
    // Constructor with full parameter initialization
    IteratedWidth(ALEInterface &sim,
                  size_t frameskip,
                  bool use_minimal_action_set,
                  size_t num_tracked_atoms,
                  int screen_features,
                  int simulator_budget,
                  float time_budget,
                  int width,
                  size_t max_rep,
                  float discount,
                  float alpha,
                  bool use_alpha_to_update_reward_for_death,
                  int nodes_threshold,
                  size_t max_depth, bool novelty_subtable, int lookahead = 10, bool printing_sketches = false)
      : SimPlanner(sim, frameskip, use_minimal_action_set, simulator_budget, num_tracked_atoms,
                   lookahead, printing_sketches),
        // Initialize configuration parameters
        width_(width),
        screen_features_(screen_features),
        time_budget_(time_budget),
        use_minimal_action_set_(use_minimal_action_set),
        max_rep_(max_rep),
        discount_(discount),
        alpha_(alpha),
        use_alpha_to_update_reward_for_death_(use_alpha_to_update_reward_for_death),
        nodes_threshold_(nodes_threshold),
        max_depth_(max_depth),
        // Initialize runtime state
        root_height_(0),
        random_decision_(false), 
        novelty_subtable(novelty_subtable){
        //initialize_sketches();
    }

    virtual ~IteratedWidth() { }

    // Accessors for planner status
    virtual bool random_decision() const { return random_decision_; }
    virtual size_t height() const { return root_height_; }
    virtual size_t expanded() const { return num_expansions_; }

    // Returns planner name with parameters
    virtual std::string name() const {
        return std::string("IW(") + std::to_string(width_) + ")";
    }

    // Main planning function - builds search tree and selects best branch
    virtual Node* get_branch(ALEInterface &env,
                             const std::vector<Action> &prefix,
                             Node *root,
                             float last_reward,
                             std::deque<Action> &branch) const {
        logging::Logger::Info << "**** IW: get branch ****" << std::endl;
        logging::Logger::Info << "prefix: sz=" << prefix.size() << ", actions=";
        print_prefix(logging::Logger::Info, prefix);
        logging::Logger::Continuation(logging::Logger::Info) << std::endl;
        logging::Logger::Info << "input:"
                    << " #nodes=" << (root == nullptr ? "na" : std::to_string(root->num_nodes()))
                    << ", #tips=" << (root == nullptr ? "na" : std::to_string(root->num_tip_nodes()))
                    << ", height=" << (root == nullptr ? "na" : std::to_string(root->height_))
                    << std::endl;
        reset_stats();
        float start_time = Utils::read_time_in_seconds();

        // Initialize root node if not provided
        if (root == nullptr) {
            Node *root_parent = new Node(nullptr, PLAYER_A_NOOP, -1);
            root_parent->state_ = new ALEState;
            apply_prefix(sim_, initial_sim_state_, prefix, root_parent->state_);
            root = new Node(root_parent, prefix.back(), 0);
        }

        // Prepare tree for search
        root->normalize_depth();
        root->reset_frame_rep_counters(frameskip_);
        root->recompute_path_rewards(root);

        // BFS exploration queue
        std::queue<Node*> queue;
        queue.push(root);
        
        if (novelty_subtable) {
            novelty_table_map_.clear();
        } else {
            novelty_table_.clear();
        }

        // Main search loop
        while (!queue.empty() && 
               (simulator_calls_ < simulator_budget_) && 
               (Utils::read_time_in_seconds() - start_time < time_budget_)) {
            Node* current = queue.front();
            queue.pop();

            // Prune terminal/solved/deep nodes
            if (current->solved_ || current->terminal_ || current->depth_ > max_depth_) {
                current->solved_ = true;
                continue;
            }

            // Expand node if needed
            expand_node(current);
            num_expansions_++;

            // Ensure current node has valid state info
            if (current->is_info_valid_ != 2) {
                update_info(current, screen_features_, alpha_, use_alpha_to_update_reward_for_death_);
            }

            // Process children
            bool all_children_solved = true;
            for (Node* child = current->first_child_; child != nullptr; child = child->sibling_) {
                // Update child state if needed
                if (child->is_info_valid_ != 2) {
                    update_info(child, screen_features_, alpha_, use_alpha_to_update_reward_for_death_);
                }

                // Generate feature tuples for novelty check
                std::vector<int> features = child->feature_atoms_;
                if (child->frame_rep_ > 0) {
                    features.push_back(child->frame_rep_);
                }
                std::sort(features.begin(), features.end());
                std::cout << "reached till here (l163)" << std::endl;
                auto tuples = generate_combinations(features, width_);
                std::cout << "reached till here (l164)" << std::endl;
                int current_logscore = 0;
                if (novelty_subtable) {
                    current_logscore = logscore(child->path_reward_);
                }

                // Novelty detection
                bool is_novel = false;
                for (const auto &tuple : tuples) {
                    bool tuple_novel = false;
                    if (novelty_subtable) {
                        tuple_novel = (novelty_table_map_[current_logscore].find(tuple) == novelty_table_map_[current_logscore].end());
                    } else {
                        tuple_novel = (novelty_table_.find(tuple) == novelty_table_.end());
                    }
                    if (tuple_novel) {
                        is_novel = true;
                        break;
                    }
                }

                // Child processing logic
                if (child->terminal_) {
                    child->solved_ = true;
                } else if (is_novel) {
                    // Add novel tuples to table
                    for (const auto &tuple : tuples) {
                        if (novelty_subtable) { novelty_table_map_[current_logscore].insert(tuple);
                        } else {  novelty_table_.insert(tuple); }
                    }
                    queue.push(child);
                    child->solved_ = false;
                    all_children_solved = false;
                } else {
                    // Prune non-novel children
                    child->solved_ = true;
                }
            }

            current->solved_ = all_children_solved;
        }

        // Value backup and branch selection
        root->backup_values(discount_);
        root->calculate_height();
        root_height_ = root->height_;
        // compute branch
        //exploitation
        //checking for the sketches: 
        if(root->parent_->screen_pixels_.size() == 0) {
            root->parent_->screen_pixels_ = root->screen_pixels_;
        }
        root->pre = check_sketches_preconditions( root->parent_->screen_pixels_,root->screen_pixels_, *this);
        // Fallback to random action if no children
        if (root->num_children_ == 0) {  
            random_decision_ = true;
            branch.push_back(random_action());
        } else {
            std::cout << "sketches exploitation with depth lookahead" << std::endl;
            logging::Logger::Info << logging::Logger::green() << "exploitation with depth lookahead" << std::endl;
            // Use depth-based sketch evaluation (e.g., look 3 steps ahead)

            const int lookahead_depth = 5; // Adjust this value as needed
            root->best_sketch_branch(branch, root->pre, lookahead_depth, discount_); 
            /* if ((!branch.empty() && branch.front() == 1)) {
                                reset_item_state(); // Reset item states if the first action is 1
                                std::cout << "Resetting item states due to action 1" << std::endl;
                        }*/
            // Fallback if no branch was found (shouldn't happen)
            if (branch.empty()) {
                logging::Logger::Info << logging::Logger::green() << "fallback to original behavior " << std::endl;
                root->best_branch(branch, discount_);
            }
        }

        // Final statistics
        total_time_ = Utils::read_time_in_seconds() - start_time;
        print_stats(logging::Logger::Stats, *root);

        return root;
    }

private:
    // Node expansion logic
    void expand_node(Node *node) const {
        if (node->num_children_ == 0) {
            assert(node->first_child_ == nullptr);

            if (node->frame_rep_ == 0) {
                // Full expansion with all actions
                ++num_expansions_;
                float start_time = Utils::read_time_in_seconds();
                node->expand(action_set_);
                expand_time_ += Utils::read_time_in_seconds() - start_time;
            } else {
                // Frame repetition expansion
                assert((node->parent_ != nullptr) && (screen_features_ > 0));
                node->expand(node->action_);
            }

            assert((node->num_children_ > 0) && (node->first_child_ != nullptr));
        }
    }

    // Combination generator for k-tuples
    std::vector<std::vector<int>> generate_combinations(const std::vector<int>& features, int k) const {
        std::vector<std::vector<int>> combinations;
        if (k == 0 || features.size() < k) return combinations;

        std::vector<int> indices(k);
        for (int i = 0; i < k; ++i) indices[i] = i;

        // Generate all k-length combinations
        do {
            std::vector<int> combination;
            for (int i = 0; i < k; ++i) {
                combination.push_back(features[indices[i]]);
            }
            std::sort(combination.begin(), combination.end());
            combinations.push_back(combination);
        } while (next_combination(indices, features.size(), k));

        return combinations;
    }

    // Combination iterator helper
    bool next_combination(std::vector<int>& indices, int n, int k) const {
        int i = k - 1;
        while (i >= 0 && indices[i] == n - k + i) --i;
        if (i < 0) return false;
        indices[i]++;
        for (int j = i + 1; j < k; ++j) {
            indices[j] = indices[j - 1] + 1;
        }
        return true;
    }

    // Reset performance counters
    void reset_stats() const {
        SimPlanner::reset_stats();
        num_expansions_ = 0;
        total_time_ = 0;
    }

    // Print search statistics
    void print_stats(logging::Logger::mode_t logger_mode, const Node &root) const {
        size_t total_novel = 0;
        if (novelty_subtable) {
            for (const auto &entry : novelty_table_map_) {
                total_novel += entry.second.size();
            }
        } else {
            total_novel = novelty_table_.size();
        }
    
        logger_mode << "IW(" << width_ << ") stats:"
                    << " #nodes=" << root.num_nodes()
                    << " #novel_tuples=" << total_novel
                    << " #expansions=" << num_expansions_
                    << " time=" << total_time_
                    << std::endl;
    }
    int logscore(float path_reward) const {
        if (path_reward <= 0) return 0;
        int logr = static_cast<int>(std::floor(std::log2(path_reward)));
        return (path_reward < 1) ? logr : 1 + logr;
    }

};

#endif // ITERATED_WIDTH_H