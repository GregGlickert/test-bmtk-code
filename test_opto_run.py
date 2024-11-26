from bmtk.simulator import bionet
from opto_stim import optoStim
from spike_module import Spiker


conf = bionet.Config.from_json('config.json', validate=True)
conf.build_env()

stim_params = {
    "nPulses": 1,
    "Dt_on": 2,
    "Dt_off": 95,
    "gmax": 0.04,
    "tauChR2": 0.4,
    "Gd1": 0.25,
    "Gd2": 0.5,
    "light_intensity": 1,
    "stimulus_position" : [0,0,0]
}

opto_module = optoStim(
    node_set="biophysical_nodes",
    section_name="soma",
    stim_params=stim_params,
    pulse_file="ramppulses.txt")

graph = bionet.BioNetwork.from_config(conf)
sim = bionet.BioSimulator.from_config(conf, network=graph)
sim.add_mod(opto_module)  
sim.run()
bionet.nrn.quit_execution()
