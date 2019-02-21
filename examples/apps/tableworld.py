from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes


def define_walls(robot):
    w_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType01, CustomObjectMarkers.Circles2, 710, 50, 25, 25, True)

    e_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType02, CustomObjectMarkers.Triangles2, 710, 50, 25, 25, True)

    n_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType03, CustomObjectMarkers.Diamonds2, 1330, 50, 25, 25, True)

    s_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType04, CustomObjectMarkers.Hexagons2, 1330, 50, 25, 25, True)

    return w_wall, e_wall, n_wall, s_wall
