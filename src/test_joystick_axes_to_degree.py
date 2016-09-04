from unittest import TestCase

from joy_to_command_translation import joystick_axes_to_degree


class TestJoystick_axes_to_degree(TestCase):
    def test_joystick_axes_to_degree(self):
        self.assertEqual(None, joystick_axes_to_degree(0, 0))
        self.assertAlmostEqual(-90, joystick_axes_to_degree(32767, 0), 2)
        self.assertAlmostEqual(-45, joystick_axes_to_degree(16767, -16767), 2)
        self.assertAlmostEqual(0, joystick_axes_to_degree(0, -32767), 2)
        self.assertAlmostEqual(45, joystick_axes_to_degree(-16767, -16767), 2)
        self.assertAlmostEqual(90, joystick_axes_to_degree(-32767, 0), 2)
        self.assertAlmostEqual(135, joystick_axes_to_degree(-16767, 16767), 2)
        self.assertAlmostEqual(180, joystick_axes_to_degree(0, 32767), 2)
