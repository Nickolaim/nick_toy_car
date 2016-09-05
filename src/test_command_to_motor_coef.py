from unittest import TestCase

from joy_to_command_translation import command_to_motor_coef


class TestCommand_to_motor_coef(TestCase):
  def test_command_to_motor_coef(self):
    self.assertEqual((0, 0), command_to_motor_coef("STOP"))
    self.assertEqual((0, 0), command_to_motor_coef("BLA"))

    self.assertEqual((1, 0), command_to_motor_coef("MOVE-90"))
    self.assertEqual((.75, .25), command_to_motor_coef("MOVE-45"))
    self.assertEqual((.5, .5), command_to_motor_coef("MOVE0"))
    self.assertEqual((.25, .75), command_to_motor_coef("MOVE45"))
    self.assertEqual((0, 1), command_to_motor_coef("MOVE90"))
