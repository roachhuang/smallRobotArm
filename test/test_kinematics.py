import unittest
import numpy as np
from robot_tools.kinematics.transforms import TransformationUtils
from robot_tools.kinematics.joint_limits import JointLimits
from robot_tools.kinematics.robotarm_class import SmallRbtArm

class TestKinematics(unittest.TestCase):
    def setUp(self):
        # Create a simple DH table for testing
        self.dh_table = np.array([
            [0, 100, 50],  # Joint 1
            [np.pi/2, 0, 0],  # Joint 2
            [0, 100, 0],  # Joint 3
            [np.pi/2, 0, 100],  # Joint 4
            [-np.pi/2, 0, 0],  # Joint 5
            [0, 0, 50]  # Joint 6
        ])
        self.robot = SmallRbtArm(self.dh_table)
        
    def test_joint_limits(self):
        """Test joint limit handling"""
        limits = JointLimits(
            max_limits=(130, 130, 74, 50, 120, 180),
            min_limits=(-130, -78, -66, -30, -90, -180)
        )
        
        # Test within limits
        angles = [0, 0, 0, 0, 0, 0]
        limited = limits.limit_joint_angles(angles)
        self.assertEqual(angles, limited)
        
        # Test exceeding limits
        angles = [150, -90, 80, 60, 130, 200]
        limited = limits.limit_joint_angles(angles)
        self.assertEqual(limited, [130, -78, 74, 50, 120, 180])
        
    def test_transformations(self):
        """Test coordinate transformations"""
        transforms = TransformationUtils()
        
        # Test pose to transformation matrix
        pose = (100, 200, 300, 30, 45, 60)
        T = transforms.pose2T(pose)
        
        # Check matrix properties
        self.assertEqual(T.shape, (4, 4))
        self.assertTrue(np.allclose(T[3], [0, 0, 0, 1]))
        
        # Test transformation matrix to pose
        recovered_pose = transforms.T2Pose(T)
        self.assertEqual(len(recovered_pose), 6)
        
    def test_forward_kinematics(self):
        """Test forward kinematics calculation"""
        # Test zero configuration
        joint_angles = (0, 0, 0, 0, 0, 0)
        T = self.robot.fk(joint_angles)
        
        # Check basic properties
        self.assertEqual(T.shape, (4, 4))
        self.assertTrue(np.allclose(T[3], [0, 0, 0, 1]))
        
        # Test non-zero configuration
        joint_angles = (45, 30, -30, 20, 10, 5)
        T = self.robot.fk(joint_angles)
        self.assertEqual(T.shape, (4, 4))
        self.assertTrue(np.allclose(T[3], [0, 0, 0, 1]))
        
    def test_inverse_kinematics(self):
        """Test inverse kinematics calculation"""
        # Test with a known configuration
        joint_angles = (30, 45, -30, 20, 10, 5)
        T = self.robot.fk(joint_angles)
        
        # Calculate IK solution
        ik_solution = self.robot.ik(T)
        
        # Verify solution exists
        self.assertIsNotNone(ik_solution)
        self.assertEqual(len(ik_solution), 6)
        
        # Verify forward kinematics of IK solution matches original pose
        T_recovered = self.robot.fk(ik_solution)
        self.assertTrue(np.allclose(T, T_recovered, atol=1e-10))

if __name__ == '__main__':
    unittest.main()