# Project for Modelling Technical Systems (Crane)

## Modelica model

The Modelica model of the Crane and the testbenches are located in the `Crane.mo` file.

## FMU Simulation environment

Folder `FMU` contains the FMU simulation environment used to run the Modelica model.

Usage:

1. Move to the `FMU` directory

    ```bash
    cd FMU
    ```

2. Start FMU environment (available on `http://localhost:81`)

    ```bash
    ./setup_scripts/start_docker.sh mts:81
    ```

3. Create conda environment

    ```bash
    conda env create -f environment.yml
    ```

4. Activate conda environment

    ```bash
    conda activate fmu_sim
    ```

5. Run the models

    ```bash
    python run_model.py
    ```

## Machine learning model

Folder `machine_learning_model` contains the machine learning model used to detect faults
Usage:

1. Move to the `machine_learning_model` directory

    ```bash
    cd machine_learning_model
    ```

2. Install dependencies

    ```bash
    pip install -r requirements.txt
    ```

3. Change the data you want to analyze by changing the path to the `fault_file` in the `detect_fault.py` file.

4. Run prediction model

    ```bash
    python detect_fault.py
    ```
