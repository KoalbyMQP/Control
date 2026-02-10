
from koalbyMove import koalbyMove

def main():

    urdf_path = 'Humanoid_URDF_9-10\\urdf\\Humanoid_URDF_9-10.urdf'
    urdf_path1 = 'Balancing_Chess_URDF\\urdf\\Balancing_Chess_URDF.urdf'

    moveRoutine = koalbyMove(urdf_path1)

    moveRoutine.motionRoutine()

if __name__ == "__main__":
    main()

