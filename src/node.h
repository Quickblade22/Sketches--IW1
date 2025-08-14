// (c) 2017 Blai Bonet

#ifndef NODE_H
#define NODE_H

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include "/usr/local/include/ale/ale_interface.hpp"
#include "logger.h"
using namespace ale;

class Node;
inline void remove_tree(Node *node);

class Node {
  public:
    
    bool visited_;                           // label
    bool solved_;                            // label
    Action action_;                          // action mapping parent to this node
    int depth_;                              // node's depth
    int height_;                             // node's height (calculated)
    float reward_;                           // reward for this node
    float path_reward_;                      // reward of full path leading to this node
    int is_info_valid_;                      // is info valid? (0=no, 1=partial, 2=full)
    bool terminal_;                          // is node a terminal node?
    float value_;                            // backed up value
    int ale_lives_;                          // remaining ALE lives
    std::vector<bool> pre; // pre sketch table for this node
    std::vector<bool> post; // post sketch table for this node
    mutable ALEState *state_;                // state for this node
    mutable std::vector<int> feature_atoms_; // features made true by this node
    mutable int num_novel_features_;         // number of features this node makes novel
    mutable int frame_rep_;                  // frame counter for number identical feature atoms through ancestors
    mutable std::vector<pixel_t> screen_pixels_; // screen pixels for this node
    // structure
    int num_children_;                       // number of children
    Node *first_child_;                      // first child
    Node *sibling_;                          // right sibling of this node
    Node *parent_;                           // pointer to parent node
    Node *grandfather;                       // pointer to previous parent node
    mutable bool node_ykeyt ; // ykeyt state for this node
    mutable bool node_bkeyt ; // bkeyt state for this node
    mutable bool node_yswrt ; // yswrt state for this node
    mutable bool node_chalicet ; // chalicet state for this node
    mutable int node_Last_room_color = 1; // 0 yellow throne, 1 yellow 2 green 3 purpel 4 red 5 light green 6 blue 7 black 8 red 9 pink
    mutable bool node_ydragon = false;
    mutable bool node_gdragon = false;
    Node(Node *parent, Action action, size_t depth)
      : visited_(false),
        solved_(false),
        action_(action),
        depth_(depth),
        height_(0),
        reward_(0),
        path_reward_(0),
        is_info_valid_(0),
        terminal_(false),
        value_(0),
        ale_lives_(-1),
        state_(nullptr),
        num_novel_features_(0),
        frame_rep_(0),
        num_children_(0),
        first_child_(nullptr),
        sibling_(nullptr),
        parent_(parent) {
        //eventually get rid of it 
        grandfather = (parent != nullptr) ? parent->parent_ : nullptr;
        node_bkeyt = (parent != nullptr) ? parent->node_bkeyt: false;
        node_ykeyt = (parent != nullptr) ? parent->node_ykeyt: false;
        node_yswrt = (parent != nullptr) ? parent->node_yswrt: false;
        node_chalicet = (parent != nullptr) ? parent->node_chalicet: false;
        node_Last_room_color = (parent != nullptr) ? parent->node_Last_room_color: 1;
        node_ydragon = (parent != nullptr) ? parent->node_ydragon: false;
        node_gdragon = (parent != nullptr) ? parent->node_gdragon: false;
    }
    ~Node() { delete state_; }
    const bool print_debug = false; // Set to true to print debug information
    const bool debug = false; 
    const bool new_iw_debug = true;
    void remove_children() {
        while( first_child_ != nullptr ) {
            Node *child = first_child_;
            first_child_ = first_child_->sibling_;
            remove_tree(child);
        }
    }
    
    void expand(Action action) {
        Node *new_child = new Node(this, action, 1 + depth_);
        //new_child->pre = this->post; 
        new_child->sibling_ = first_child_;
        first_child_ = new_child;
        ++num_children_;
        new_child->grandfather = this->parent_;
        new_child->node_ykeyt = this->node_ykeyt;
        new_child->node_bkeyt = this->node_bkeyt;
        new_child->node_yswrt = this->node_yswrt;
        new_child->node_chalicet = this->node_chalicet;
        new_child->node_Last_room_color = this->node_Last_room_color;
        new_child->node_ydragon = this->node_ydragon;
        new_child->node_gdragon = this->node_gdragon;
    }
    void expand(const ActionVect &actions, bool random_shuffle = true) {
        assert((num_children_ == 0) && (first_child_ == nullptr));
        for( size_t k = 0; k < actions.size(); ++k )
            expand(actions[k]);
        //if( random_shuffle ) std::random_shuffle(children_.begin(), children_.end()); // CHECK: missing
        assert(num_children_ == int(actions.size()));
    }

    void clear_cached_states() {
        if( is_info_valid_ == 2 ) {
            delete state_;
            state_ = nullptr;
            is_info_valid_ = 1;
        }
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            child->clear_cached_states();
    }

    Node* advance(Action action) {
        assert((num_children_ > 0) && (first_child_ != nullptr));
        assert((parent_ == nullptr) || (parent_->parent_ == nullptr));
        if( parent_ != nullptr ) {
            delete parent_;
            parent_ = nullptr;
        }

        Node *sibling = nullptr;
        Node *selected = nullptr;
        for( Node *child = first_child_; child != nullptr; child = sibling ) {
            sibling = child->sibling_;
            if( child->action_ == action )
                selected = child;
            else
                remove_tree(child);
        }
        assert(selected != nullptr);

        selected->sibling_ = nullptr;
        first_child_ = selected;
        return selected;
    }

    void normalize_depth(int depth = 0) {
        depth_ = depth;
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            child->normalize_depth(1 + depth);
    }

    void reset_frame_rep_counters(int frameskip, int parent_frame_rep) {
        if( frame_rep_ > 0 ) {
            frame_rep_ = parent_frame_rep + frameskip;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
                child->reset_frame_rep_counters(frameskip, frame_rep_);
        }
    }
    void reset_frame_rep_counters(int frameskip) {
        reset_frame_rep_counters(frameskip, -frameskip);
    }

    void recompute_path_rewards(const Node *ref = nullptr) {
        if( this == ref ) {
            path_reward_ = 0;
        } else {
            assert(parent_ != nullptr);
            path_reward_ = parent_->path_reward_ + reward_;
        }
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            child->recompute_path_rewards();
    }

    void solve_and_backpropagate_label() {
        assert(!solved_);
        if( !solved_ ) {
            solved_ = true;
            if( parent_ != nullptr ) {
                assert(!parent_->solved_);
                bool unsolved_siblings = false;
                for( Node *child = parent_->first_child_; child != nullptr; child = child->sibling_ ) {
                    if( !child->solved_ ) {
                        unsolved_siblings = true;
                        break;
                    }
                }
                if( !unsolved_siblings )
                    parent_->solve_and_backpropagate_label();
            }
        }
    }

    float qvalue(float discount) const {
        return reward_ + discount * value_;
    }

    float backup_values_upward(float discount) { // NOT USED
        assert((num_children_ == 0) || (is_info_valid_ != 0));

        value_ = 0;
        if( num_children_ > 0 ) {
            assert(first_child_ != nullptr);
            float max_child_value = -std::numeric_limits<float>::infinity();
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                float child_value = child->qvalue(discount);
                max_child_value = std::max(max_child_value, child_value);
            }
            value_ = max_child_value;
        }

        if( parent_ == nullptr )
            return value_;
        else
            return parent_->backup_values_upward(discount);
    }

    float backup_values(float discount) {
        assert((num_children_ == 0) || (is_info_valid_ != 0));
        value_ = 0;
        if( num_children_ > 0 ) {
            assert(first_child_ != nullptr);
            float max_child_value = -std::numeric_limits<float>::infinity();
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                child->backup_values(discount);
                float child_value = child->qvalue(discount);
                max_child_value = std::max(max_child_value, child_value);
            }
            value_ = max_child_value;
        }
        return value_;
    }

    float backup_values_along_branch(const std::deque<Action> &branch, float discount, size_t index = 0) { // NOT USED
        //assert(is_info_valid_ && !children_.empty()); // tree now grows past branch
        if( index == branch.size() ) {
            backup_values(discount);
            return value_;
        } else {
            assert(index < branch.size());
            float value_along_branch = 0;
            const Action &action = branch[index];
            float max_child_value = -std::numeric_limits<float>::infinity();
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( child->action_ == action )
                    value_along_branch = child->backup_values_along_branch(branch, discount, ++index);
                max_child_value = std::max(max_child_value, child->value_);
            }
            value_ = reward_ + discount * max_child_value;
            return reward_ + discount * value_along_branch;
        }
    }

    const Node *best_tip_node(float discount) const { // NOT USED
        if( num_children_ == 0 ) {
            return this;
        } else {
            assert(first_child_ != nullptr);
            size_t num_best_children = 0;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
                num_best_children += child->qvalue(discount) == value_;
            assert(num_best_children > 0);
            size_t index_best_child = lrand48() % num_best_children;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( child->qvalue(discount) == value_ ) {
                    if( index_best_child == 0 )
                        return child->best_tip_node(discount);
                    --index_best_child;
                }
            }
            assert(0);
        }
    }

    void best_branch(std::deque<Action> &branch, float discount) const {
        if( num_children_ > 0 ) {
            assert(first_child_ != nullptr);
            size_t num_best_children = 0;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
                num_best_children += child->qvalue(discount) == value_;
            assert(num_best_children > 0);
            size_t index_best_child = lrand48() % num_best_children;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( child->qvalue(discount) == value_ ) {
                    if( index_best_child == 0 ) {
                        branch.push_back(child->action_);
                        child->best_branch(branch, discount);
                        break;
                    }
                    --index_best_child;
                }
            }
        }
    }
    int evaluate_sketch_potential(const std::vector<bool>& pres, int depth_limit) const {
        int count = 0;
        for (size_t i = 0; i < post.size(); ++i) {
           if(print_debug)logging::Logger::Info << logging::Logger::green() <<" "<<i << ")"<< " pre: " << pres[i] << " post: " << post[i] << std::endl;
            if (post[i] && pres[i]) ++count;
        }
        if (depth_limit == 0 || num_children_ == 0) return count;

        int max_potential = count;
        for (Node* child = first_child_; child != nullptr; child = child->sibling_) {
            int child_potential = child->evaluate_sketch_potential(pres, depth_limit - 1);
            if(print_debug) logging::Logger::Info << logging::Logger::green() << "Max_potential: " << max_potential << " child_potential: " << child_potential << " +count: "<< count << std::endl;
            max_potential = std::max(max_potential, count + child_potential);
           
        }
        return max_potential;
    }
    // Evaluates sketch potential up to a certain depth
    int evaluate_sketch_potentials(const std::vector<bool>& target_sketch, int depth_limit) const {
        if (depth_limit <= 0) {
            // Base case: evaluate this node's sketch fulfillment
            int count = 0;
            if(print_debug) logging::Logger::Info << logging::Logger::green()<< "evaluate_sketch_potentials" << std::endl; 
            for (size_t i = 0; i < post.size(); ++i) {
                if(print_debug) logging::Logger::Info << logging::Logger::green()<< i << ") " << " pre " << target_sketch[i] << " post: " << post[i] << std::endl;
                if (post[i] && target_sketch[i]) {
                    ++count;
                    
                }
            }
            return count;
        }
        
        // Recursive case: look at children
        int max_potential = 0;
        for (Node *child = first_child_; child != nullptr; child = child->sibling_) {
            int child_potential = child->evaluate_sketch_potential(target_sketch, depth_limit - 1);
           if(print_debug) logging::Logger::Info << logging::Logger::green() << "Max_potential: " << max_potential << " child_potential: " << child_potential << std::endl;
            max_potential = std::max(max_potential, child_potential);

        }
        return max_potential;
    }
    Node* select_child_by_sketch_potential(const std::vector<bool>& target_sketch, int lookahead_depth, float discount,int current_priority) const {
        assert(num_children_ > 0 && first_child_ != nullptr);
        
        std::vector<Node*> candidate_children;
        // Fallback to all children if no candidates
        if (candidate_children.empty()) {
            for (Node *child = first_child_; child != nullptr; child = child->sibling_) {
                candidate_children.push_back(child);
            }
        }

        // Select best child from candidates
        int best_potential = -1;
        float best_qvalue = -std::numeric_limits<float>::infinity();
        std::vector<Node*> best_children;
        
        for (Node *child : candidate_children) {
            int potential = child->evaluate_sketch_potential(target_sketch, lookahead_depth);
            float qval = child->qvalue(discount);
            
            if (potential > best_potential || 
            (potential == best_potential && qval > best_qvalue)) {
                if(new_iw_debug) logging::Logger::Info << logging::Logger::green() 
                    << "Better child found: "<< best_potential << " <= " << potential
                    << " reward:"<< best_qvalue << " <= "<< qval<< std::endl;
                best_potential = potential;
                best_qvalue = qval;
                best_children.clear();
                best_children.push_back(child);
            } else if (potential == best_potential && qval == best_qvalue) {
                best_children.push_back(child);
            }
        }
        
        assert(!best_children.empty());
        if (best_children.size() == 1) {
            if(new_iw_debug) logging::Logger::Info << logging::Logger::green() 
                << "found one child" << std::endl;
            return best_children[0];
        } else {
            if(new_iw_debug) logging::Logger::Info << logging::Logger::green() 
                << "Better child found random: "<< best_potential 
                << "reward:"<< best_qvalue << std::endl;
            return best_children[lrand48() % best_children.size()];
        }
    }
    // Builds best branch based on sketch potential
    void best_sketch_branch(std::deque<Action>& branch, const std::vector<bool>& target_sketch, int lookahead_depth, float discount,int priority = 0,int action_nr=0) const {
        // Try to find shortest path to sketch fulfillment
    if (debug) std::cout << "Best_sketch_branch called with depth: " << lookahead_depth << std::endl;
        auto branchs = find_sketch_fulfillment_path(target_sketch, lookahead_depth); //resets the item states if action 1 is found
            if(new_iw_debug) std::cout << "found a path "<< branchs.size() << std::endl;
            if (!branchs.empty()) {
                branch.insert(branch.begin(), branch.end(), branchs.begin()); // Insert current action at the start
                if (new_iw_debug) {
                for(auto i: branch) {
                    std::cout << "Action in branch: " << i << std::endl;
                }
                }
        }

        if(new_iw_debug) std::cout << "falling back to orignal selection of child in Node " << lookahead_depth << std::endl;
        if (num_children_ > 0  ) { //&& lookahead_depth > 0
            Node* best_child = select_child_by_sketch_potential(target_sketch, lookahead_depth, discount,priority);
            
            branch.push_back(best_child->action_);
            if(new_iw_debug) std::cout << std::endl << "Best child action_nr: " << action_nr << " action:" << best_child->action_  << std::endl;
            
        }
        
    }
   
    std::deque<Action> find_sketch_fulfillment_path(const std::vector<bool>& target_sketch, int max_depth) const {
        // Use BFS to find shortest path to sketch-fulfilling node
        int depth = max_depth;
        std::queue<std::pair<const Node*, std::deque<Action>>> q;
        q.push({this, {}});
        
        while (!q.empty() && max_depth >= 0) {
            int level_size = q.size();
            for (int i = 0; i < level_size; i++) {
                auto [current, path] = q.front();
                q.pop();
                
                // Check if current node fulfills any sketch
                int fulfilled_count = 0;
                if( print_debug ) std::cout << "Checking node at depth: " << current->depth_ << std::endl;
                for (size_t idx = 0; idx < current->post.size(); idx++) {
                    
                    if (target_sketch[idx] && current->post[idx]) {
                        if(new_iw_debug) {
                            std::cout << "Checking sketch index: " << idx 
                                  << " post: " << current->post[idx] 
                                  << " target: " << target_sketch[idx] << std::endl;
                        }
                        fulfilled_count++;
                    }
                }
                
                // Return path if any sketch is fulfilled
                if (fulfilled_count > 0) {
                    if(new_iw_debug) std::cout <<"this was the max_depth " << max_depth << " searched till " << depth << std::endl;
                    return path;
                }
                
                // Enqueue children with updated path
                for (Node* child = current->first_child_; child; child = child->sibling_) {
                    std::deque<Action> new_path = path;
                    new_path.push_back(child->action_);
                    q.push({child, new_path});
                }
            }
            depth--;
        }
        if(new_iw_debug) std::cout <<" return 0 path this was the max_depth " << max_depth << " searched till " << depth << std::endl;
        return {}; // No path found
    }
    
    
    void longest_zero_value_branch(float discount, std::deque<Action> &branch) const {
        assert(value_ == 0);
        if( num_children_ > 0 ) {
            assert(first_child_ != nullptr);
            size_t max_height = 0;
            size_t num_best_children = 0;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( (child->qvalue(discount) == 0) && (child->height_ >= int(max_height)) ) {
                    if( child->height_ > int(max_height) ) {
                        max_height = child->height_;
                        num_best_children = 0;
                    }
                    ++num_best_children;
                }
            }
            assert(num_best_children > 0);
            size_t index_best_child = lrand48() % num_best_children;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( (child->qvalue(discount) == 0) && (child->height_ == int(max_height)) ) {
                    if( index_best_child == 0 ) {
                        branch.push_back(child->action_);
                        child->longest_zero_value_branch(discount, branch);
                        break;
                    }
                    --index_best_child;
                }
            }
        }
    }

    size_t num_tip_nodes() const {
        if( num_children_ == 0 ) {
            return 1;
        } else {
            assert(first_child_ != nullptr);
            size_t n = 0;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
                n += child->num_tip_nodes();
            return n;
        }
    }

    size_t num_nodes() const {
        size_t n = 1;
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            n += child->num_nodes();
        return n;
    }

    int calculate_height() {
        height_ = 0;
        if( num_children_ > 0 ) {
            assert(first_child_ != nullptr);
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                int child_height = child->calculate_height();
                height_ = std::max(height_, child_height);
            }
            height_ += 1;
        }
        return height_;
    }

    void print_branch(std::ostream &os, const std::deque<Action> &branch, size_t index = 0) const {
        print(os);
        if( index < branch.size() ) {
            Action action = branch[index];
            bool child_found = false;
            for( Node *child = first_child_; child != nullptr; child = child->sibling_ ) {
                if( child->action_ == action ) {
                    child->print_branch(os, branch, ++index);
                    child_found = true;
                    break;
                }
            }
            assert(child_found);
        }
    }

    void print(std::ostream &os) const {
        os << "node:"
           << " valid=" << is_info_valid_
           << ", solved=" << solved_
           << ", lives=" << ale_lives_
           << ", value=" << value_
           << ", reward=" << reward_
           << ", path-reward=" << path_reward_
           << ", action=" << action_
           << ", depth=" << depth_
           << ", children=[";
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            os << child->value_ << " ";
        os << "] (this=" << this << ", parent=" << parent_ << ")"
           << std::endl;
    }

    void print_tree(std::ostream &os) const {
        print(os);
        for( Node *child = first_child_; child != nullptr; child = child->sibling_ )
            child->print_tree(os);
    }
    void set_presketch_table(std::vector<bool> pre_sketch_table) {
        pre = pre_sketch_table;
    }
    void set_postsketch_table(std::vector<bool> post_sketch_table) {
        post = post_sketch_table;
    }
    std::vector<bool> get_presketch_table() {
        return pre;
    }
    std::vector<bool> get_postsketch_table() {
        return post;
    }
};

inline void remove_tree(Node *node) {
    Node *sibling = nullptr;
    for( Node *child = node->first_child_; child != nullptr; child = sibling ) {
        sibling = child->sibling_;
        remove_tree(child);
    }
    delete node;
}

#endif

