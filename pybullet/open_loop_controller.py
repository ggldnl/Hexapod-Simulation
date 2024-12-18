#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloise Dalin , eloise.dalin@inria.fr
#| Pierre Desreumaux , pierre.desreumaux@inria.fr
#|
#| Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret.
#|"Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
import numpy as np
import math


class OpenLoopController:
    '''
        Implement an open-loop controller based on periodic signals
        Please see the supplementary information of Cully et al., Nature, 2015
    '''
    def __init__(self, params, array_dim=100):
        self.array_dim = array_dim
        self.trajs = np.zeros(1)

    def step(self, t):
        assert(self.trajs.shape[0] != 1)
        k = int(math.floor(t * self.array_dim)) % self.array_dim
        return self.trajs[:, k]

    def _control_signal(self, amplitude, phase, duty_cycle, array_dim=100):
        '''
        create a smooth periodic function with amplitude, phase, and duty cycle,
        amplitude, phase and duty cycle are in [0, 1]
        '''
        assert(amplitude >= 0 and amplitude <= 1)
        assert(phase >= 0 and phase <= 1)
        assert(duty_cycle >= 0 and duty_cycle <= 1)
        command = np.zeros(array_dim)

        # create a 'top-hat function'
        up_time = array_dim * duty_cycle
        temp = [amplitude if i < up_time else -amplitude for i in range(0, array_dim)]

        # smoothing kernel
        kernel_size = int(array_dim / 10)
        kernel = np.zeros(int(2 * kernel_size + 1))
        sigma = kernel_size / 3
        for i in range(0, len(kernel)):
            kernel[i] =  math.exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma**2)) / (sigma * math.sqrt(math.pi))
        sum = np.sum(kernel)

        # smooth the function
        for i in range(0, array_dim):
            command[i] = 0
            for d in range(1, kernel_size + 1):
                if i - d < 0:
                    command[i] += temp[array_dim + i - d] * kernel[kernel_size - d]
                else:
                    command[i] += temp[i - d] * kernel[kernel_size - d]
            command[i] += temp[i] * kernel[kernel_size]
            for d in range(1, kernel_size + 1):
                if i + d >= array_dim:
                    command[i] += temp[i + d - array_dim] * kernel[kernel_size + d]
                else:
                    command[i] += temp[i + d] * kernel[kernel_size + d]
            command[i] /= sum

        # shift according to the phase
        final_command = np.zeros(array_dim)
        start = int(math.floor(array_dim * phase))
        current = 0
        for i in range(start, array_dim):
            final_command[current] = command[i]
            current += 1
        for i in range(0, start):
            final_command[current] = command[i]
            current += 1

        assert(len(final_command) == array_dim)
        return final_command
