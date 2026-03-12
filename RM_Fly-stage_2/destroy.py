import airsim
client = airsim.MultirotorClient()
names = client.simListSceneObjects()
print(names)


client.simDestroyObject("MovingCircle_2")       # circle 1
client.simDestroyObject("MovingCircle2_5")      # circle 2
client.simDestroyObject("MovingCircle3_11")     # circle 3
client.simDestroyObject("MovingCircle4_2")      # circle 4
client.simDestroyObject("3PointsCircle5_14")    # circle 5
client.simDestroyObject("3PointsCircle_2")      # circle 6
client.simDestroyObject("3PointsCircle2_5")     # circle 7
client.simDestroyObject("3PointsCircle3_8")     # circle 8
client.simDestroyObject("3PointsCircle4_11")    # circle 9
client.simDestroyObject("3PointsCircle8")       # circle 10
client.simDestroyObject("3PointsCircle6_17")    # circle 11
client.simDestroyObject("MovingCircle6_2")      # circle 12
client.simDestroyObject("MovingCircle5_17")     # circle 13
client.simDestroyObject("GameEnd_2")            # circle 14
client.simDestroyObject("CN_MI_Drone")
client.simDestroyObject("Movingobstacles_2")    # blue obs
client.simDestroyObject("obstacles2_5")         # red car
client.simDestroyObject("obstacles3")           # white car
client.simDestroyObject("obstacles_2")          # blue car
