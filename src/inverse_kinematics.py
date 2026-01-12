import opensim as osim
import numpy as np

"""Inverse kinematics simulation run using the non-amputated model. The model came from OpenSim's 3D walking example. It is a version of the Rajagopal (2016) 3D model."""

# This code is adapted from https://github.com/opensim-org/opensim-core/blob/main/Bindings/Python/examples/Moco/example3DWalking/exampleMocoTrack.py

def inverse_kinematics():
    # Construct the MocoInverse tool.
    inverse = osim.MocoInverse()

    # Construct a ModelProcessor and set it on the tool. The default
    # muscles in the model are replaced with optimization-friendly
    # DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    # parameters.
    modelProcessor = osim.ModelProcessor('models/not_amputated_model.osim')
    modelProcessor.append(osim.ModOpAddExternalLoads('sto/grf_walk.xml'))
    # Replace the PinJoints representing the model's toes with WeldJoints.
    jointsToWeld = osim.StdVectorString()
    jointsToWeld.append("mtp_r")
    jointsToWeld.append("mtp_l")
    modelProcessor.append(osim.ModOpReplaceJointsWithWelds(jointsToWeld))
    # Add CoordinateActuators to the pelvis coordinates. 
    modelProcessor.append(osim.ModOpAddResiduals(250.0, 50.0, 1.0))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

    model = modelProcessor.process()
    state = model.initSystem()
    model.equilibrateMuscles(state)  # if applicable
    modelProcessor = osim.ModelProcessor(model)

    inverse.setModel(modelProcessor)

    # Construct a TableProcessor of the coordinate data and pass it to the
    # inverse tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    tableProcessor = osim.TableProcessor('sto/coordinates_not_amputated.sto')
    inverse.setKinematics(tableProcessor)

    for i in range(model.getForceSet().getSize()):
        f = model.getForceSet().get(i)
        if "Bushing" in f.getConcreteClassName():
            b = osim.BushingForce.safeDownCast(f)
            values = b.getRecordValues(state)
            py_values = [values.get(j) for j in range(values.getSize())]               # Convert to Python list
            print(py_values)

    # joint = model.getJointSet().get(f"joint_segment_1")
    # coord = joint.get_coordinates(0)
    # print(coord.getDefaultValue())  # initial angle


    # Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.48)
    inverse.set_final_time(1.61)
    inverse.set_mesh_interval(0.02)
    inverse.set_convergence_tolerance(1e-2)
    inverse.set_constraint_tolerance(1e-2)

    # By default, Moco gives an error if the kinematics contains extra columns.
    # Here, we tell Moco to allow (and ignore) those extra columns.
    inverse.set_kinematics_allow_extra_columns(True)

    # Solve the problem and write the solution to a Storage file.
    solution = inverse.solve()

    try:
        solution.getMocoSolution().write('output_not_amputated_inv_kin.sto')

        # Generate a PDF with plots for the solution trajectory.
        model = modelProcessor.process()
        report = osim.report.Report(model,
                                    'output_not_amputated_inv_kin.sto',
                                    bilateral=True)
        
        # The PDF is saved to the working directory.
        report.generate()
    except:
        print('Solver failed.')
    
if __name__ == "__main__":
    inverse_kinematics()