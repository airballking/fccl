# name of the constraint used for identification in knowledge base
string name

# name of tf-frame w.r.t. which the constraint is defined, i.e. usually the
# frame from which the relation is 'seen'
string frame_id

# name of the function used to relate the two features
string function

# toolFeature which HAS to be connected to the kinematic chain controlled by
# the controller
Feature tool_feature

# objectFeature which does not need to be connected to the kinematic chain, but
# but can be
Feature object_feature

# lower bound for desired output of function --the motion goal
float64 lower_boundary 

# upper bound for desired output of function --the motion goal
float64 upper_boundary 
