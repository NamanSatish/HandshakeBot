import rospy
import json
from geometry_msgs.msg import PointStamped


class ProcedureStep:
    def __init__(self, axis_x=False, axis_y=False, axis_z=False, duration=0):
        self.axis_x = axis_x  # True for mirror, False for track
        self.axis_y = axis_y  # True for mirror, False for track
        self.axis_z = axis_z  # True for mirror, False for track
        self.duration = duration

    def to_dict(self):
        return {
            "axis_x": self.axis_x,
            "axis_y": self.axis_y,
            "axis_z": self.axis_z,
            "duration": self.duration,
        }

    @staticmethod
    def from_dict(data):
        return ProcedureStep(
            axis_x=data.get("axis_x", False),
            axis_y=data.get("axis_y", False),
            axis_z=data.get("axis_z", False),
            duration=data.get("duration", 0),
        )


class Procedure:
    def __init__(self, steps=None):
        self.steps = steps if steps else []

    def to_json(self, file_path):
        with open(file_path, "w") as f:
            json.dump({"steps": [step.to_dict() for step in self.steps]}, f, indent=4)

    @staticmethod
    def from_json(file_path):
        with open(file_path, "r") as f:
            data = json.load(f)
            steps = [ProcedureStep.from_dict(step) for step in data.get("steps", [])]
            return Procedure(steps)

    @staticmethod
    def create_procedure_interactively():
        print("Interactive Procedure Creation")
        steps = []
        while True:
            print("\nCreating a new step...")
            axis_x = input("Mirror X axis? (yes/no): ").strip().lower()
            axis_x = axis_x == "yes" or axis_x == "y"
            axis_y = input("Mirror Y axis? (yes/no): ").strip().lower()
            axis_y = axis_y == "yes" or axis_y == "y"
            axis_z = input("Mirror Z axis? (yes/no): ").strip().lower()
            axis_z = axis_z == "yes" or axis_z == "y"
            duration = float(input("Enter duration (seconds): ").strip())

            steps.append(
                ProcedureStep(
                    axis_x=axis_x, axis_y=axis_y, axis_z=axis_z, duration=duration
                )
            )

            cont = input("Add another step? (yes/no): ").strip().lower()
            if cont != "yes":
                break

        file_path = input(
            "Enter file name to save the procedure (e.g., procedure.json): "
        ).strip()
        procedure = Procedure(steps)
        procedure.to_json(file_path)
        print(f"Procedure saved to {file_path}")


class HandshakeProcedure:
    def __init__(self, json_file):
        rospy.init_node("handshake_procedure")

        self.ar_tag_origin = None
        self.hand_coord_sub = rospy.Subscriber(
            "hand_pose_base", PointStamped, self.hand_callback
        )
        self.robot_point_pub = rospy.Publisher(
            "robot_point", PointStamped, queue_size=1
        )

        self.current_hand_position = None
        self.procedure = Procedure.from_json(json_file)

    def hand_callback(self, msg):
        self.current_hand_position = msg

    def get_ar_tag_origin(self):
        rospy.wait_for_message("hand_pose_base", PointStamped)
        while True:
            response = input(
                f"Current Location is {self.current_hand_position.point.x},{self.current_hand_position.point.y},{self.current_hand_position.point.z}, if correct type y"
            )
            if response == "y":
                self.ar_tag_origin = self.current_hand_position
                rospy.loginfo(f"Origin set to: {self.ar_tag_origin}")
                break

    def mirror_and_track(self, step):
        if not self.ar_tag_origin or not self.current_hand_position:
            rospy.logwarn("Missing AR tag origin or hand position. Skipping this step.")
            return

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        while rospy.Time.now() - start_time < rospy.Duration(step.duration):
            try:
                target_point = PointStamped()
                target_point.header.frame_id = "base"
                target_point.header.stamp = rospy.Time.now()
                target_point.point = self.current_hand_position.point

                if step.axis_x:  # Mirror
                    target_point.point.x = (
                        2 * self.ar_tag_origin.point.x - target_point.point.x
                    )
                else:  # Track
                    target_point.point.x = self.current_hand_position.point.x

                if step.axis_y:  # Mirror
                    target_point.point.y = (
                        2 * self.ar_tag_origin.point.y - target_point.point.y
                    )
                else:  # Track
                    target_point.point.y = self.current_hand_position.point.y

                if step.axis_z:  # Mirror
                    target_point.point.z = (
                        2 * self.ar_tag_origin.point.z - target_point.point.z
                    )
                else:  # Track
                    target_point.point.z = self.current_hand_position.point.z

                self.robot_point_pub.publish(target_point)
                rate.sleep()
            except KeyboardInterrupt:
                break

    def execute_procedure(self):
        for step in self.procedure.steps:
            rospy.loginfo(
                f"Executing step with settings: {step.to_dict()} for {step.duration} seconds"
            )
            self.mirror_and_track(step)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: rosrun your_package_name script_name.py <path_to_json_file>")
        print(
            "Or run 'rosrun your_package_name script_name.py interactive' to create a procedure interactively."
        )
        sys.exit(1)

    arg = sys.argv[1]
    if arg.lower() == "interactive":
        Procedure.create_procedure_interactively()
    else:
        json_file = arg
        handshake_procedure = HandshakeProcedure(json_file)

        handshake_procedure.get_ar_tag_origin()

        rospy.sleep(1)  # Allow some time for initialization

        try:
            handshake_procedure.execute_procedure()
        except rospy.ROSInterruptException:
            pass
