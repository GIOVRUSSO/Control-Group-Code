import sys

import torch
from tensordict.tensordict import TensorDict
from torchrl.data.replay_buffers import ReplayBuffer, LazyTensorStorage
from torchrl.data.replay_buffers.samplers import SliceSampler


import pickle
class Buffer:
	"""
	Replay buffer for TD-MPC2 training. Based on torchrl.
	Uses CUDA memory if available, and CPU memory otherwise.
	"""

	def __init__(self, cfg):
		self.cfg = cfg
		if sys.platform == "darwin":
			self._device = torch.device("cpu")
		else:
			self._device = torch.device("cuda")
		self._capacity = min(cfg.buffer_size, cfg.steps)
		self._sampler = SliceSampler(
			num_slices=self.cfg.batch_size, # (horizon: 3 + 1) * batch_size: 256
			end_key=None,
			traj_key="episode",
			truncated_key=None,
		)
		self._batch_size = cfg.batch_size * (cfg.horizon + 1)
		self._num_eps = 0

	@property
	def capacity(self):
		"""Return the capacity of the buffer."""
		return self._capacity

	@property
	def num_eps(self):
		"""Return the number of episodes in the buffer."""
		return self._num_eps

	def _reserve_buffer(self, storage):
		"""
		Reserve a buffer with the given storage.
		"""
		return ReplayBuffer(
			storage=storage,
			sampler=self._sampler,
			pin_memory=True,
			prefetch=1,
			batch_size=self._batch_size,
		)

	def _init(self, tds):
		"""Initialize the replay buffer. Use the first episode to estimate storage requirements."""
		print(f"Buffer capacity: {self._capacity:,}")
		if sys.platform == "darwin":
			mem_free = 0
		else:
			mem_free, _ = torch.cuda.mem_get_info()
		bytes_per_step = sum(
			[
				(
					v.numel() * v.element_size()
					if not isinstance(v, TensorDict)
					else sum([x.numel() * x.element_size() for x in v.values()])
				)
				for v in tds.values()
			]
		) / len(tds)
		total_bytes = bytes_per_step * self._capacity
		print(f"Storage required: {total_bytes/1e9:.2f} GB")
		# Heuristic: decide whether to use CUDA or CPU memory
		storage_device = "cuda" if 2.5 * total_bytes < mem_free else "cpu"
		print(f"Using {storage_device.upper()} memory for storage.")
		return self._reserve_buffer(
			LazyTensorStorage(self._capacity, device=torch.device(storage_device))
		)

	def _to_device(self, *args, device=None):
		if device is None:
			device = self._device
		return (
			arg.to(device, non_blocking=True) if arg is not None else None
			for arg in args
		)

	def _prepare_batch(self, td):
		"""
		Prepare a sampled batch for training (post-processing).
		Expects `td` to be a TensorDict with batch size TxB.
		"""
		obs = td["obs"]
		action = td["action"][1:]
		reward = td["reward"][1:].unsqueeze(-1)
		task = td["task"][0] if "task" in td.keys() else None
		return self._to_device(obs, action, reward, task)

	def add(self, td):
		"""Add an episode to the buffer."""
		td["episode"] = torch.ones_like(td["reward"], dtype=torch.int64) * self._num_eps

		# FIX for HumanoidBench #
		if len(td["episode"]) <= self.cfg.horizon + 1:
			return self._num_eps
		################################

		if self._num_eps == 0:
			self._buffer = self._init(td)
		self._buffer.extend(td)
		self._num_eps += 1
		return self._num_eps

	def sample(self):
		"""Sample a batch of subsequences from the buffer."""
		td = self._buffer.sample().view(-1, self.cfg.horizon + 1).permute(1, 0)
		return self._prepare_batch(td)

	def save(self, path):
		"""Save the buffer to a file."""

		# Save the buffer to a file.

		buffer_path = path.replace(".pkl", "_buffer.pkl")
		self._buffer.dumps(buffer_path)
		print(f"_buffer saved to {buffer_path}")

		save_dict = {
			"_num_eps": self._num_eps,
			"_capacity": self._capacity,
			"_batch_size": self._batch_size,
			"_device": self._device,
			"cfg": self.cfg,
		}
		with open(path, "wb") as f:
			pickle.dump(save_dict, f)
		
		print(f"Buffer Object saved to {path}")

	def load(self, path):
		"""Load the buffer from a file."""
		with open(path, "rb") as f:
			load_dict = pickle.load(f)
		self._buffer = self._reserve_buffer(
			LazyTensorStorage(self._capacity, device=self._device)
		)
		
		# Load the buffer from a file.
		buffer_path = path.replace(".pkl", "_buffer.pkl")
		self._buffer.loads(buffer_path)

		self._num_eps = load_dict["_num_eps"]
		self._capacity = load_dict["_capacity"]
		self._batch_size = load_dict["_batch_size"]
		self._device = load_dict["_device"]
		self.cfg = load_dict["cfg"]
		print(f"Buffer loaded from {path}")
		return self._num_eps