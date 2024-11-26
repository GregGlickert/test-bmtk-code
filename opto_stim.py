from bmtk.simulator.bionet.modules.sim_module import SimulatorMod
from neuron import h
import numpy as np
import h5py
import os

pc = h.ParallelContext()
MPI_RANK = int(pc.id())
N_HOSTS = int(pc.nhost())


class optoStim(SimulatorMod):
    def __init__(self, node_set, section_name, section_index=0, section_dist=0.5, stim_params=None, pulse_file=None,
                 output_file="opto_stim_currents.h5", tmp_dir="./tmp"):
        """
        Initialize the optoStim module.

        :param node_set: Node set name or ID to apply stimulation.
        :param section_name: The name of the section (e.g., 'soma').
        :param section_index: Index of the section in the structure (default is 0).
        :param section_dist: Location along the section (0.0 to 1.0, default is 0.5).
        :param stim_params: Dictionary of stimulation parameters.
        :param pulse_file: File containing pulse times (one pulse per line).
        :param output_file: The HDF5 file to save final currents.
        :param tmp_dir: Directory for storing temporary files.
        """
        self._node_set = node_set
        self._section_name = section_name
        self._section_index = section_index
        self._section_dist = section_dist
        self._stim_params = stim_params or {}
        self._pulse_file = pulse_file
        self._pulse_times = None
        self._output_file = output_file if os.path.isabs(output_file) else os.path.join(tmp_dir, output_file)
        self._tmp_dir = tmp_dir
        self._tmp_file = self._get_tmp_fname(MPI_RANK)
        self._tmp_handle = None
        self._block_data = None
        self._gids_on_rank = None

        if pulse_file:
            self._load_pulse_file(pulse_file)

        self._stims = []
        self._netcons = []
        self._vecstims = []
        self.currents = []
        
        if os.path.isfile(self._output_file):
            os.remove(self._output_file)

    def _get_tmp_fname(self, rank):
        """Generate a unique temporary filename for each rank."""
        return os.path.join(self._tmp_dir, f'tmp_{rank}_{os.path.basename(self._output_file)}')

    def _create_tmp_file(self, nsteps, nsites):
        """Create a temporary HDF5 file for the current rank."""
        os.makedirs(self._tmp_dir, exist_ok=True)
        self._tmp_handle = h5py.File(self._tmp_file, 'w')
        self._tmp_handle.create_dataset(
            '/stim/data', (nsteps, nsites), dtype=np.float64, chunks=True, maxshape=(None, nsites)
        )
        self._tmp_handle.create_dataset('/stim/gids', (nsites,), dtype=np.int32)  # Store GIDs
        self._tmp_handle['/stim/gids'][:] = self._gids_on_rank  # Save GIDs
        self._tmp_handle.flush()

    def _write_block_data(self, block_interval):
        """Write current block data to the temporary file."""
        start_step = block_interval[0]
        end_step = block_interval[1]
        self._tmp_handle['/stim/data'][start_step:end_step, :] = self._block_data[start_step:end_step, :]
        self._block_data[start_step:end_step, :] = 0
        self._tmp_handle.flush()

    def _combine_files(self, nsteps, nsites):
        """Combine all temporary files into the final output file."""
        if MPI_RANK == 0:
            with h5py.File(self._output_file, 'w') as out_file:
                out_file.create_dataset('/stim/data', (nsteps, nsites * N_HOSTS), dtype=np.float64, chunks=True)
                out_file.create_dataset('/stim/gids', (nsites * N_HOSTS,), dtype=np.int32)  # Combined GIDs
                for rank in range(N_HOSTS):
                    tmp_file = self._get_tmp_fname(rank)
                    with h5py.File(tmp_file, 'r') as tmp_handle:
                        start_idx = rank * nsites
                        end_idx = (rank + 1) * nsites
                        out_file['/stim/data'][:, start_idx:end_idx] = tmp_handle['/stim/data'][:]
                        out_file['/stim/gids'][start_idx:end_idx] = tmp_handle['/stim/gids'][:]
        pc.barrier()

    def _load_pulse_file(self, pulse_file):
        """Load pulse times from a text file."""
        try:
            self._pulse_times = np.loadtxt(pulse_file)
        except Exception as e:
            raise ValueError(f"Error loading pulse file '{pulse_file}': {e}")

    def initialize(self, sim):
        """Called once at the beginning of the simulation."""
        select_gids = list(sim.net.get_node_set(self._node_set).gids())
        self._gids_on_rank = list(set(select_gids) & set(sim.local_gids))

        nsteps = int(sim.n_steps)
        nsites = len(self._gids_on_rank)
        self._create_tmp_file(nsteps, nsites)
        self._block_data = np.zeros((nsteps, nsites))

        for gid in self._gids_on_rank:
            cell = sim.net.get_cell_gid(gid)
            hobj_sec = getattr(cell.hobj, self._section_name)[self._section_index](self._section_dist)
            self._create_stim(hobj_sec, cell)

    def step(self, sim, tstep):
        """Called at each simulation time step."""
        current_values = [vec.x[-1] for vec in self.currents]
        self._block_data[tstep, :] = current_values

    def block(self, sim, block_interval):
        """Called every block."""
        self._write_block_data(block_interval)

    def finalize(self, sim):
        """Called at the end of the simulation."""
        self._tmp_handle.close()
        pc.barrier()
        self._combine_files(sim.n_steps, len(self.currents))
        if MPI_RANK == 0:
            for rank in range(N_HOSTS):
                tmp_file = self._get_tmp_fname(rank)
                if os.path.exists(tmp_file):
                    os.remove(tmp_file)

    def _get_stimulus_distance(self, cell):
        """Calculate the Euclidean distance between the stimulus and cell soma."""
        stim_location = np.array(self._stim_params.get("stimulus_position", [0, 0, 0]))
        cell_location = np.array(cell.soma_position)
        return np.linalg.norm(stim_location - cell_location)

    def _create_stim(self, hobj_sec, cell):
        """Create the optogenetic stimulation for the specified section."""
        stim = h.ChR2_william_event(hobj_sec)
        distance = self._get_stimulus_distance(cell)
        max_intensity = self._stim_params.get("light_intensity")
        scaled_intensity = max_intensity / (1 + (distance) / 10)
        stim.nPulses = self._stim_params.get("nPulses")
        stim.Dt_on = self._stim_params.get("Dt_on")
        stim.Dt_off = self._stim_params.get("Dt_off")
        stim.gmax = self._stim_params.get("gmax")
        stim.tauChR2 = self._stim_params.get("tauChR2")
        stim.Gd1 = self._stim_params.get("Gd1")
        stim.Gd2 = self._stim_params.get("Gd2")
        stim.light_intensity = scaled_intensity
        self._stims.append(stim)

        i = h.Vector().record(stim._ref_i)
        self.currents.append(i)

        pulse_vec = h.Vector(self._pulse_times)
        vec_stim = h.VecStim()
        vec_stim.play(pulse_vec)
        self._vecstims.append(vec_stim)

        netcon = h.NetCon(vec_stim, stim)
        netcon.weight[0] = 1
        netcon.delay = 0
        self._netcons.append(netcon)
