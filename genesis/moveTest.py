
from koalbyMove import koalbyMove

def main():

    urdf_path = 'Humanoid_URDF_9-10//urdf//Humanoid_URDF_9-10.urdf'

    moveRoutine = koalbyMove(urdf_path)

    moveRoutine.motionRoutine()

if __name__ == "__main__":
    main()

