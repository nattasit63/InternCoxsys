from vrp import VRPPD

vrppd = VRPPD()


def main():
    open = vrppd.open_config()
    adjMatrix = vrppd.adjacency_matrix()
    solver = vrppd.vrppd_solver(max_station=7,capacity=5)
    astar_path = vrppd.a_star_for_vehicle()
    return astar_path
if __name__=="__main__":
    print(main())