from collections import deque
import numpy as np

class CraneVar:
    """
    Variable for the Crane class. Next to keeping a history of past values,
    it calculates an Exponential Moving Average (EMA) and two derivatives:
    a raw derivative and a smoothed derivative.

    Attributes
    ema_exponent     (float) : sensitivity of EMA
    history          (deque) : fixed length array
    n_hist           (int)   : length of history deque
    value            (float) : raw value
    value_smooth     (float) : (EMA) smoothed value over `n_hist` values
    value_d          (float) : 1st order derivative
    value_d_smooth   (float) : (EMA) smoothed 1st order derivative over `n_hist-1` values
    """
    def __init__(self, n_hist=50, ema_exponent=6.):
        self.value = 0.
        self.n_hist = n_hist
        self.history = deque([0.]*self.n_hist,maxlen=self.n_hist)
        self.history_smooth = deque([0.]*self.n_hist,maxlen=self.n_hist)

        # EMA
        self.value_smooth = 0.
        self._ema_exponent = ema_exponent # the higher this value, the faster it adapts.
        self._ema_weights = np.logspace(0., self._ema_exponent, self.n_hist)
        self._ema_weights /= np.sum(self._ema_weights)

        # derivative
        self.value_d = 0.
        self.value_d_smooth = 0.

    def update(self, new_value: float):
        '''Update the variable + bookkeeping.'''
        self.value = new_value
        self.history.append(new_value)

        # EMA
        self.value_smooth = np.sum(np.asarray(self.history)*self._ema_weights)
        self.history_smooth.append(self.value_smooth)

        # derivative
        self.value_d = self.history[-1]-self.history[-2]
        # calculate the differences between values, this new array will be of length (n_hist-1)
        # history_d = np.asarray(self.history)[1:]-np.asarray(self.history)[:self.n_hist-1]

        # EMA derivative
        self.value_d_smooth = self.history_smooth[-1]-self.history_smooth[-2]