import opensim as osim

def convert_muscles_to_degroote(model=None, save_model=False):
    """
    Convert muscles in an OpenSim model from Millard2012EquilibriumMuscle
    to DeGrooteFregly2016Muscle using a ModelProcessor. 
    """
    if model is None:
        # -------- USER SETTINGS --------
        input_model = "models/new_model.osim"
        output_model = "models/prosthesisModel_7.osim"
        # --------------------------------

        print("Loading model:", input_model)
        model = osim.Model(input_model)
    
    model.initSystem()

    # Set minimum muscle controls and activations to 0 (default is 0.01).
    muscles = model.updMuscles()
    for imuscle in range(muscles.getSize()):
        muscle = osim.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(imuscle))
        muscle.setMinimumActivation(0.0)
        muscle.setMinControl(0.0)

    mp = osim.ModelProcessor(model)

    mp.append(osim.ModOpIgnoreTendonCompliance())
    mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

    print("Applying model processing operations...")
    processed = mp.process()
    processed.finalizeConnections()

    if save_model is True:
        # export the processed model
        processed.printToXML(output_model)
        print(f"Saved processed model to: {output_model}")
    
    return processed

if __name__ == "__main__":
    convert_muscles_to_degroote(save_model=True)