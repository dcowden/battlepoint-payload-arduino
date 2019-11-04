nominal_speed = [ 100, 180, 250, -100, -255 ]

gain = 0.2

#(left,right)
sensor_inputs = [ 
    ("no reading",0,0), 
    ("left_very_close", 700,0), 
    ("right_very_close",0,700), 
    ("left_close",100,0),
    ("right_close",0,100),
    ("both_close",100,100) ]

def compute_gain (speed, inputs ):

    delta = inputs[1] - inputs[2]
    MAX_SPEED = 255
    correction = delta*gain
    if speed > 0:
        leftSpeed = min(speed - correction,MAX_SPEED)
        rightSpeed = min(speed + correction,MAX_SPEED)        
    else:
        leftSpeed = max(speed + correction,-MAX_SPEED)
        rightSpeed = max(speed - correction,-MAX_SPEED)        

    
    printrow( (inputs[0],speed, leftSpeed,rightSpeed) )


def printrow(r):
    w = 20
    print str(r[0]).ljust(w),str(r[1]).ljust(w),str(r[2]).ljust(w),str(r[3]).ljust(w)
    
printrow( ("case","nominal_speed","leftcommand","rightcommand"))
for s in nominal_speed:
    for i in sensor_inputs:
        compute_gain(s,i)