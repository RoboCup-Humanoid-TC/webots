kick={}

kick.bodyHeight = 0.41696;

kick.def={};

kick.def["kickForwardLeft"]={
    supportLeg = 1,
        def = {
        {1, 0.8, {-0.01,-0.05,0} ,kick.bodyHeight}, -- COM slide
        {2, 0.2, {-0.01,-0.065,0} , {-0.05,-0.02,0},0.05 , 0}, -- Lifting
        {2, 0.2, {-0.01,-0.075,0} , {-0.07,0.025,0} ,0.08 , 10*math.pi/180, 20*math.pi/180}, -- Lifting
        {4, 0.2, {-0.025,-0.065,0} , {0.5,0.045,0}  ,0.1 , -40*math.pi/180, -6*math.pi/180},-- Kicking
        {2, 0.2,{-0.015,-0.05,0} , {-0.6,0,0} ,0.008, 0*math.pi/180,  30*math.pi/180}, -- Landing
        {1, 0.1, {0.00,-0.020, 0}}, -- COM slide
        {6, 0.9, {0.000,-0.01, 0} , 20*math.pi/180}, -- Stabilize   
    },
};

return kick