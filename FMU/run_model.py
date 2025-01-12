import interface.api_fmu_docker.api_lib as api
import numpy as np
import csv

testbenches = [
    {
        "name": "Top spring breaks",
        "config": "models/02_top_spring_breaks.json",
        "output_file": "output_top_spring_breaks.csv",
        "duration": 80,
    },
    # {
    #     "name": "Bottom spring breaks",
    #     "config": "models/03_bottom_spring_breaks.json",
    #     "output_file": "output_bottom_spring_breaks.csv",
    #     "duration": 80,
    # },
    # {
    #     "name": "Top wire breaks",
    #     "config": "models/04_top_wire_breaks.json",
    #     "output_file": "output_top_wire_breaks.csv",
    #     "duration": 80,
    # },
    # {
    #     "name": "Bottom wire breaks",
    #     "config": "models/05_bottom_wire_breaks.json",
    #     "output_file": "output_bottom_wire_breaks.csv",
    #     "duration": 80,
    # },
]


def init(config_file):
    api_model = api.API_FMU_Docker("model")

    # Load configuration file
    api_model.init_interface_config_client(config_file)

    # Upload FMU file
    api_model.upload_fmu()

    # Upload FMU file
    api_model.upload_config()

    # Reset FMU model
    api_model.send_reset()

    return api_model


def simulate(tb, model: api.API_FMU_Docker):

    writer = csv.writer(open(tb["output_file"], "w"))
    writer.writerow(
        ["time", "sut.crane_angle", "sut.control_force", "sut.desired_angle", "sut.crane_wire_join.measured_weight"]
    )

    for i in np.arange(0, tb["duration"], model.get_timestep()):
        model.send_update()

        output = model.get_output()

        writer.writerow(
            [
                output["time"]["value"],
                output["sut.crane_angle"]["value"],
                output["sut.control_force"]["value"],
                output["sut.desired_angle"]["value"],
                output["sut.crane_wire_join.measured_weight"]["value"],
            ]
        )


if __name__ == "__main__":

    for tb in testbenches:

        print(f"Running testbench: {tb['name']}")

        model = init(tb["config"])
        simulate(tb, model)
