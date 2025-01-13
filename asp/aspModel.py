import os
import csv
import clingo

program = \
"""
velocity(T, V) :- measuredAngle(T, A1), measuredAngle(T+1, A2), V = A2-A1.
acceleration(T, A) :- velocity(T, V1), velocity(T+1, V2), A = V2-V1.

% Component state basics
state(springTop, ok) :- not state(springTop, broken).
:- state(springTop, ok), state(springTop, broken).
state(springBottom, ok) :- not state(springBottom, broken).
:- state(springBottom, ok), state(springBottom, broken).
state(wireTop, ok) :- not state(wireTop, broken).
:- state(wireTop, ok), state(wireTop, broken).
state(wireBottom, ok) :- not state(wireBottom, broken).
:- state(wireBottom, ok), state(wireBottom, broken).
state(angleSensor, ok) :- not state(angleSensor, broken).
:- state(angleSensor, ok), state(angleSensor, broken).
state(weightSensor, ok) :- not state(weightSensor, broken).
:- state(weightSensor, ok), state(weightSensor, broken).

% The measured weight should change if the crane changes acceleration accelerates
state(weightSensor, broken) :- acceleration(T, A1), acceleration(T+1, A2), A1 != A2, measuredWeight(T, W1), measuredWeight(T, W2), W1 == W2.

% The angle should change when the forces of the crane change
force(T, down) :- desiredForceTop(T, F1), desiredForceTop(T+1, F2), F2 < F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 <= W2, velocity(T, V), V <= 0.
force(T, down) :- desiredForceBottom(T, F1), desiredForceBottom(T+1, F2), F2 > F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 <= W2, velocity(T,V), V <= 0.
force(T, up) :- desiredForceTop(T, F1), desiredForceTop(T+1, F2), F2 > F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 >= W2, velocity(T,V), V <= 0.
force(T, up) :- desiredForceBottom(T, F1), desiredForceBottom(T+1, F2), F2 < F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 >= W2, velocity(T,V), V <= 0.
state(angleSensor, broken) :- force(T, down), acceleration(T, A), A >= 0.
state(angleSensor, broken) :- force(T, up), acceleration(T, A), A <= 0.

% If weight does not increase, but the force, then the acceleration should increase
1{state(wireTop, broken); state(springTop, broken)}2 :- desiredForceTop(T, F1), F1 > 0, desiredForceTop(T+1, F2), F2 > F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 >= W2, acceleration(T, A1), acceleration(T+1, A2), A1 >= A2.

% If weight does not decrease, but the force does, then the acceleration should decrease
1{state(wireBottom, broken); state(springBottom, broken)}2 :- desiredForceBottom(T, F1), F1 < 0, desiredForceBottom(T+1, F2), F2 < F1, measuredWeight(T, W1), measuredWeight(T+1, W2), W1 <= W2, acceleration(T, A1), acceleration(T+1, A2), A1 <= A2.

% Output diagnostic results
output_fault(Comp, Fault) :- state(Comp, Fault), Fault != ok.

% Finding minimal fault sets
#minimize { 1@1, Comp : output_fault(Comp, _) }.

#show output_fault/2.
"""

def write_asp_input(parameters, timestep):
    """Generate ASP input facts based on the given parameters."""
    asp_facts = []
    for key, value in parameters.items():
        value = int(float(value) * 1000)

        if key == "time":
            continue
        if key == "sut.crane_angle":
            key = "measuredAngle"
        if key == "sut.desired_angle":
            key = "desiredAngle"
        if key == "sut.crane_wire_join.measured_weight":
            key = "measuredWeight"
        if key == "sut.control_force":
            key = "desiredForceTop"
            bottom_value = -value if value < 0 else 0
            asp_facts.append(f"desiredForceBottom({timestep}, {bottom_value}).")
            if value < 0:
                value = 0

        asp_facts.append(f"{key}({timestep}, {value}).")
    return "\n".join(asp_facts)

def run_clingo_with_python_api(model_content, asp_input):
    """Run the ASP model using the clingo Python API and return the output."""
    asp_file_name = "model.pl"
    with open(asp_file_name,"w") as file:
        file.write(model_content)

    control = clingo.Control()
    control.load(asp_file_name)
    control.add("base", [], asp_input)
    control.ground([("base", [])])

    results = []

    def on_model(model):
        # Directly collect the clingo symbols from the model
        results.extend(model.symbols(shown=True))

    control.solve(on_model=on_model)
    return results

def parse_clingo_output(output):
    """Parse the clingo output symbols and extract fault diagnoses."""
    diagnoses = []
    for symbol in output:
        # Parse only the `output_fault` objects with valid arguments
        if symbol.name == "output_fault" and len(symbol.arguments) == 2:
            diagnoses.append({
                "component": str(symbol.arguments[0]),
                "fault": str(symbol.arguments[1])
            })
    return diagnoses

def process_csv(file_path):
    """Read the CSV file and process each timestep."""
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        fieldnames = reader.fieldnames

        asp_time_step = []
        output = []
        window_size = 4

        # Prepare ASP input and run the model
        for timestep, row in enumerate(reader):
            asp_time_step.append(write_asp_input(row, timestep))
            #row["Diagnosis"] = diagnosis_str

        for index in range(1, len(asp_time_step)-window_size):
            asp_input = "\n".join(asp_time_step[index:index+window_size])
            clingo_output = run_clingo_with_python_api(program, asp_input)

            # Parse and format the diagnosis
            diagnosis = parse_clingo_output(clingo_output)
            diagnosis_str = "; ".join(
                f"{item['component']}={item['fault']}" for item in diagnosis
            )

            output.append(f"{index}: {diagnosis_str}")

        return output


process_csv("parameters.csv")
