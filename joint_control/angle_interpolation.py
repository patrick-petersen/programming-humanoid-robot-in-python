'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello

firstTime = 0 # does not work the way intendentt
timeSet = False
class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
	
    
    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes)
        self.target_joints.update(target_joints)
        return super(PIDAgent, self).think(perception)

    def angle_interpolation(self, keyframes):
        target_joints = {}
	global firstTime
	global timeSet
        # YOUR CODE HERE
        self.currentTime = self.perception.time
        if timeSet == False:
            firstTime = self.currentTime
  	    timeSet = True	    
        (names, times, keys) = keyframes

        timeInKeyframes = self.currentTime - firstTime

	maxTimesForNames = 0
        for nameIndex in range(0, len(names)):
            #iterare through all joints
            name = names[nameIndex]
            timesForName = times[nameIndex]
            keysForName = keys[nameIndex]

            #prevTime = timesForName[0]
            #nextTime = timesForName[0]
            i = 0
            while i < len(timesForName)-2 and timeInKeyframes > timesForName[i]:
		i += 1
              
	    P0 = (timesForName[i], keysForName[i][0])
 	    P1 = (timesForName[i] + keysForName[i][2][1], keysForName[i][0] + keysForName[i][2][2])
            #P2 ist out of index range for some reason when len(timeForName)-1
	    P2 = (timesForName[i+1], keysForName[i+1][0]) #
            P3 = (timesForName[i+1] + keysForName[i+1][1][1], keysForName[i+1][0] + keysForName[i+1][1][2]) 
	    t = timeInKeyframes	    
	    k = timesForName[len(timesForName)-1]
	    # normally the right time intervall border, but time is in this case not element of [0,1]
	    # k should be the last Keyframe, i think
	    target_joints[name] = (k-t) ** 3 * P0[1] + 3*(k-t) ** 2 * t * P1[1] + 3 * (k-t) * t ** 2 * P2[1] + t**3 * P3[1]
	    if timesForName[len(timesForName)-1]> maxTimesForNames:
		maxTimesForNames = timesForName[len(timesForName)-1]
	# also, don't we just need on of the coordinates, because the second would be the time, wich we have
	#1-t mit 1 = k wie oben
        #(1-t) ** 3 * P0 + 3*(1-t) ** 2 * t * P1 + 3 * (1-t) * t ** 2 * P2 + t**3 * P3
	
	if timeInKeyframes > maxTimesForNames:
	    timeSet = False
	print target_joints
 	return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
