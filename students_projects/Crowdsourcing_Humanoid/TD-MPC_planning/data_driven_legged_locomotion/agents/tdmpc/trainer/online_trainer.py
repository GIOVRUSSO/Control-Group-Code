from time import time

import numpy as np
import torch
from tensordict.tensordict import TensorDict

from trainer.base import Trainer

class OnlineTrainer(Trainer):
	"""Trainer class for single-task online TD-MPC2 training."""

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self._step = 0
		self._ep_idx = 0
		self._start_time = time()
		self._starting_step = 0
		self._buffer_loaded = False

	def set_step(self, step,old_csv=None):
		"""Set the current step. Used for loading checkpoints."""
		self._step = step
		self._starting_step = step
		# Load the previous metrics
		self.logger.load_eval(old_csv)
	
	def save_buffer(self):
		"""Save the replay buffer."""
		save_pickle_path = str(self.logger.experiment_dir) + "/buffer.pkl" # Consider using os.path.join(self.logger.experiment_dir, "buffer.pkl")
		print("Saving buffer to", save_pickle_path)

		self.buffer.save(save_pickle_path)
		print("Buffer saved")

	def load_buffer(self, path):
		"""Load the replay buffer."""
		print("Loading buffer from", path)
		self._ep_idx = self.buffer.load(path)

		self._buffer_loaded = True
		print("Buffer loaded")

	def common_metrics(self):
		"""Return a dictionary of current metrics."""
		return dict(
			step=self._step,
			episode=self._ep_idx,
			total_time=time() - self._start_time,
		)

	def eval(self):
		"""Evaluate a TD-MPC2 agent."""
		ep_rewards, ep_successes = [], []
		for i in range(self.cfg.eval_episodes):
			obs, done, ep_reward, t = self.env.reset()[0], False, 0, 0
			if self.cfg.save_video:
				self.logger.video.init(self.env, enabled=(i == 0))
			while not done:
				action = self.agent.act(obs, t0=t == 0, eval_mode=True)
				obs, reward, done, truncated, info = self.env.step(action)
				done = done or truncated
				ep_reward += reward
				t += 1
				if self.cfg.save_video:
					self.logger.video.record(self.env)
			ep_rewards.append(ep_reward)
			ep_successes.append(info["success"])
			if self.cfg.save_video:
				# self.logger.video.save(self._step)
				self.logger.video.save(self._step, key='results/video')
		return dict(
			episode_reward=np.nanmean(ep_rewards),
			episode_success=np.nanmean(ep_successes),
		)

	def to_td(self, obs, action=None, reward=None):
		"""Creates a TensorDict for a new episode."""
		if isinstance(obs, dict):
			obs = TensorDict(obs, batch_size=(), device="cpu")
		else:
			obs = obs.unsqueeze(0).cpu()
		if action is None:
			action = torch.full_like(self.env.rand_act(), float("nan"))
		if reward is None:
			reward = torch.tensor(float("nan"))
		td = TensorDict(
			dict(
				obs=obs,
				action=action.unsqueeze(0),
				reward=reward.unsqueeze(0),
			),
			batch_size=(1,),
		)
		return td

	def train(self):
		"""Train a TD-MPC2 agent."""
		train_metrics, done, eval_next, save_next = {}, True, True, False

		print("[DEBUG: online_trainer.py] buffer_loaded:",self._buffer_loaded)

		while self._step <= self.cfg.steps:
			# Evaluate agent periodically
			if self._step % self.cfg.eval_freq == 0:
				eval_next = True
				
			# Save chekpoints periodically
			if self._step % self.cfg.save_freq == 0:
				save_next = True

			# Reset environment
			if done:				

				if save_next:
					self.logger.save_agent(self.agent, f"step-{self._step}")
					
					if self._step != self._starting_step:
						print("Saving buffer at step", self._step)
						self.save_buffer()
					save_next = False
					print("Saved agent at step", self._step)
					
				
				if eval_next:
					eval_metrics = self.eval()
					eval_metrics.update(self.common_metrics())
					self.logger.log(eval_metrics, "eval") # Save the metrics in the csv file
					eval_next = False

				if self._step > 0 and self._step != self._starting_step:
					train_metrics.update(
						episode_reward=torch.tensor(
							[td["reward"] for td in self._tds[1:]]
						).sum(),
						episode_success=info["success"],
					)
					train_metrics.update(self.common_metrics())

					results_metrics = {'return': train_metrics['episode_reward'],
									   'episode_length': len(self._tds[1:]),
									   'success': train_metrics['episode_success'],
									   'success_subtasks': info['success_subtasks'],
									   'step': self._step,}
				
					self.logger.log(train_metrics, "train")
					self.logger.log(results_metrics, "results")
					self._ep_idx = self.buffer.add(torch.cat(self._tds))

				#print("1. Resetting environment... Saving tensor dict")
				obs = self.env.reset()[0]
				self._tds = [self.to_td(obs)]

			# Collect experience
			if self._step > self.cfg.seed_steps or not(self.cfg.from_scratch):
			#if (self._step >= self.cfg.seed_steps and self.cfg.from_scratch) or ((self._step >= self._starting_step+self.cfg.seed_steps) and not(self.cfg.from_scratch)):
				
				# if self._step % 5000 == 0:
				# 	print("[DEBUG: online_trainer.py] acting as agent and not random act")
				action = self.agent.act(obs, t0=len(self._tds) == 1)
				#print("action:",action)
			else:
				
				# if self._step % 500 == 0:
				# 	print("[DEBUG: online_trainer.py] RANDOM ACT")
				action = self.env.rand_act()
				#print("random action:",action)
			obs, reward, done, truncated, info = self.env.step(action)
			done = done or truncated
			self._tds.append(self.to_td(obs, action, reward))

			# Update agent # If the buffer is loaded from a file, it is not necessary adding elements to the buffer, without updating the agent.
			#if (self._step >= self.cfg.seed_steps) and (self._step >= (self._starting_step+self.cfg.seed_steps+self.cfg.fill_buffer_steps)): # If from_scratch is True, _starting_step is 0.
			#	if self._step == self.cfg.seed_steps or (self._step == (self._starting_step+self.cfg.seed_steps+self.cfg.fill_buffer_steps)):# and not self.cfg.from_scratch:
			if ((self._step >= self.cfg.seed_steps) and (self._step >= (self._starting_step+self.cfg.seed_steps))) or (self._buffer_loaded): # If from_scratch is True, _starting_step is 0.
				if self._step == self.cfg.seed_steps and self.cfg.from_scratch: #TODO(my-rice): Consider removing the "and self.cfg.from_scratch" part.
					num_updates = self.cfg.seed_steps
					print("Pretraining agent (on seed data) ...")
				else:
					num_updates = 1
				for _ in range(num_updates):
					_train_metrics = self.agent.update(self.buffer)
				# if self._step % 1000 == 0:
				# 	print("[DEBUG: online_trainer.py] ... updating agent ... ")
				train_metrics.update(_train_metrics)

			self._step += 1

		self.logger.finish(self.agent)
