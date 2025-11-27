import opensim as osim

# -------- USER SETTINGS --------
input_model = "models/new_model.osim"
output_model = "models/prosthesisModel_1.osim"
# --------------------------------

print("Loading model:", input_model)
model = osim.Model(input_model)

# ---------------------------------------------------------------------
# 1. Build a ModelProcessor for converting Millard → DeGroote muscles.
# ---------------------------------------------------------------------
mp = osim.ModelProcessor(model)

mp.append(osim.ModOpIgnoreTendonCompliance())
mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

print("Applying model processing operations...")
processed = mp.process()
processed.initSystem()

# # ---------------------------------------------------------------------
# # 2. Add Bhargava 2004 Smoothed Metabolic Cost *after* muscle replacement
# # ---------------------------------------------------------------------
# print("Adding Bhargava 2004 metabolics...")
# met = osim.Bhargava2004SmoothedMuscleMetabolics()
# met.setName("metabolic_cost")
# met.set_use_smoothing(True)

# muscles = processed.getMuscles()
# for i in range(muscles.getSize()):
#     m = osim.DeGrooteFregly2016Muscle.safeDownCast(muscles.get(i))
#     if m is None:
#         raise RuntimeError("Muscle replacement did not produce DGF muscles as expected.")
#     met.addMuscle(m.getName(), m)

# processed.addComponent(met)
processed.finalizeConnections()

# ---------------------------------------------------------------------
# 3. EXPORT new model
# ---------------------------------------------------------------------
processed.printToXML(output_model)
print("\n-----------------------------------------------------")
print("Saved processed model to:")
print("   ", output_model)
print("-----------------------------------------------------\n")