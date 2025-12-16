from nav_msgs.msg import OccupancyGrid


def mappoint_to_coords(mappoint: list[int], map_data: OccupancyGrid):
    goal_x = mappoint[1] * map_data.info.resolution + map_data.info.origin.position.x
    goal_y = mappoint[0] * map_data.info.resolution + map_data.info.origin.position.y
    return [goal_x, goal_y]

def coords_to_mappoint(coord: list[float], map_data: OccupancyGrid) -> list[int]:
    if len(coord) < 2:
        return []
    coord_x = round((coord[0] - map_data.info.origin.position.x) / map_data.info.resolution)
    coord_y = round((coord[1] - map_data.info.origin.position.y) / map_data.info.resolution)
    return [coord_y, coord_x]