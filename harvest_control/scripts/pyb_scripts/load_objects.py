import numpy as np


class LoadObjects:
    def __init__(self, con) -> None:
        """ Object loader class

        Args:
            con (class): PyBullet client - an instance of the started env
        """
        self.con = con
        self.load_objects()

    def load_urdf(self, urdf_name, start_pos=[0, 0, 0], start_orientation=[0, 0, 0], fix_base=True, radius=None):
        """ Load a urdf using PyBullets urdf loader

        Args:
            urdf_name (str): filename/path to urdf file
            start_pos (list, optional): starting origin. Defaults to [0, 0, 0].
            start_orientation (list, optional): starting orientation. Defaults to [0, 0, 0].
            fix_base (bool, optional): fixed or floating. Defaults to True.
            radius (float, optional): radius of loaded object. Defaults to None.

        Returns:
            int: PyBullet object ID
        """
        orientation = self.con.getQuaternionFromEuler(start_orientation)
        if radius is None:
            objectId = self.con.loadURDF(urdf_name, start_pos, orientation, useFixedBase=fix_base)
        else:
            objectId = self.con.loadURDF(urdf_name, start_pos, globalScaling=radius, useFixedBase=fix_base)
            self.con.changeVisualShape(objectId, -1, rgbaColor=[0, 1, 0, 1]) 
        return objectId

    def load_objects(self):
        """ Load objects into the started PyBullet simulation
        """
        self.planeId = self.load_urdf("plane.urdf")

        # Define the dimensions of the rectangular prism
        length = 2.0  # x-axis (length)
        thickness = 0.1  # y-axis (thickness)
        height = 2.0  # z-axis (height)

        # Create a collision shape for the rectangular prism
        prism_collision_shape = self.con.createCollisionShape(
            shapeType=self.con.GEOM_BOX,
            halfExtents=[length / 2, thickness / 2, height / 2]  # half-extents in x, y, z
        )

        # Create a visual shape for the rectangular prism (optional)
        prism_visual_shape = self.con.createVisualShape(
            shapeType=self.con.GEOM_BOX,
            halfExtents=[length / 2, thickness / 2, height / 2],
            rgbaColor=[0.8, 0.3, 0.3, 1]  # Red color with full opacity
        )

        # Set the base position and orientation for the rectangular prism
        base_position = [0, 0.85, 1]  # Adjust the y-axis offset if needed
        base_orientation = self.con.getQuaternionFromEuler([np.deg2rad(18.435), 0, 0])

        # Create the rectangular prism in the simulation
        # self.vtrellis_treeId = self.con.createMultiBody(
        #     baseMass=0,  # 0 mass makes it static
        #     baseCollisionShapeIndex=prism_collision_shape,
        #     baseVisualShapeIndex=prism_visual_shape,
        #     basePosition=base_position,
        #     baseOrientation=base_orientation
        # )

        self.start_x = 0.5
        self.start_y = 1
        self.prune_point_0_pos = [self.start_x, self.start_y, 1.55] 
        self.prune_point_1_pos = [self.start_x, self.start_y - 0.05, 1.1] 
        self.prune_point_2_pos = [self.start_x, self.start_y + 0.05, 0.55] 
        self.radius = 0.05 

        # self.leader_branchId = self.load_urdf("./urdf/leader_branch.urdf", [0, self.start_y, 1.6/2])
        # self.top_branchId = self.load_urdf("./urdf/secondary_branch.urdf", [0, self.start_y, 1.5], [0, np.pi / 2, 0])
        # self.mid_branchId = self.load_urdf("./urdf/secondary_branch.urdf", [0, self.start_y, 1], [0, np.pi / 2, 0])
        # self.bottom_branchId = self.load_urdf("./urdf/secondary_branch.urdf", [0, self.start_y, 0.5], [0, np.pi / 2, 0])
        # self.collision_objects = [self.leader_branchId, self.top_branchId, self.mid_branchId, self.bottom_branchId, self.planeId]
        self.collision_objects = [self.planeId]#, self.vtrellis_treeId]

        # self.prune_point_0 = self.load_urdf("sphere2.urdf", self.prune_point_0_pos, radius=self.radius)
        # self.prune_point_1 = self.load_urdf("sphere2.urdf", self.prune_point_1_pos, radius=self.radius)
        # self.prune_point_2 = self.load_urdf("sphere2.urdf", self.prune_point_2_pos, radius=self.radius)
