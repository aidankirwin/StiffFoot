import opensim as osim
m = osim.Model('models/prosthesisModel_7.osim')
state = m.initSystem()

for i in range(m.getMuscles().getSize()):
    mus = osim.DeGrooteFregly2016Muscle.safeDownCast(m.getMuscles().get(i))
    name = mus.getName()
    opt_len = mus.getOptimalFiberLength()
    tendon_len = mus.getTendonSlackLength()
    # compute current fiber length & velocity (requires computeCurrentFiberLength / getLengthInfo equivalents)
    Lmt = mus.getLength(state)  # total MTU length; method name may vary
    # If the API doesn't expose getLength, use muscle.computeFiberLength() via a muscle-specific method
    fiber_length = mus.getFiberLength(state)           # may need to adapt to exact API
    fiber_vel = mus.getFiberVelocity(state)
    print(name, 'opt', opt_len, 'fiber_len', fiber_length, 'norm_len', fiber_length/opt_len, 'vel', fiber_vel)
