#!/usr/bin/env python

# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
from scipy.interpolate import RegularGridInterpolator
from matplotlib import pyplot as plt

fs = 32000  # sample rate, Hz
T = 30.  # sample duration, seconds
freqs = [12333, 10533]  # Hz

rng = np.random.RandomState(23)

def get_signal(F, fs, T):
    t = np.arange(fs * T) / fs
    weight_sum = 0
    signal = np.zeros(t.shape)

    weight = 1.0
    weight_sum += weight
    signal += weight * np.sin(2 * np.pi * F * t)

    weight = 1.5
    weight_sum += weight
    signal += weight * np.clip(1.0 / 3 * rng.normal(size=t.shape), -1, 1)

    return (32768 / weight_sum * signal).astype('int16')

for F in freqs:
    signal = get_signal(F, fs, T)
    fname = 'test_sounds/tone%s.wav' % F
    wavfile.write(fname, fs, signal)
    print('wrote %s' % fname)

for F in freqs:
    fname = 'test_sounds/tone%s.wav' % F
    fs, signal = wavfile.read(fname)

    welch_F, Pxx = welch(signal, fs, nperseg=8192, average='median')
    plt.semilogy(welch_F, Pxx)

    interp = RegularGridInterpolator([welch_F], Pxx)

    print('PSD for %s:' % fname)
    for F in freqs:
        print('  @ %s Hz: %s' % (F, interp([F])))

plt.legend([str(F) for F in freqs])
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD ($V^2$ / Hz)')

fname = 'tones.png'
plt.savefig(fname)
print('wrote %s' % fname)
plt.close()
