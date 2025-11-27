import opensim as osim

def convert_muscles_to_degroote():
    """
    Convert muscles in an OpenSim model from Millard2012EquilibriumMuscle
    to DeGrooteFregly2016Muscle using a ModelProcessor. 
    """
    # -------- USER SETTINGS --------
    input_model = "models/new_model.osim"
    output_model = "models/prosthesisModel_3.osim"
    # --------------------------------

    print("Loading model:", input_model)
    model = osim.Model(input_model)

    mp = osim.ModelProcessor(model)

    mp.append(osim.ModOpIgnoreTendonCompliance())
    mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

    print("Applying model processing operations...")
    processed = mp.process()
    processed.initSystem()

    # processed.addComponent(met)
    processed.finalizeConnections()

    # export the processed model
    processed.printToXML(output_model)
    print(f"Saved processed model to: {output_model}")

if __name__ == "__main__":
    convert_muscles_to_degroote()