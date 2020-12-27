# Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def linear_regression(xvals, yvals):
    '''Calculate the linear regression measured in the least squares sense
    for the sample points.

      y = a + b * x

    Return a tuple of (a, b)  which is (intercept, slope)
    '''

    # From: https://en.wikipedia.org/wiki/Simple_linear_regression
    assert len(xvals) == len(yvals)

    mean_x = sum(xvals) / len(xvals)
    mean_y = sum(yvals) / len(yvals)

    b = (sum((x - mean_x) * (y - mean_y) for x, y in zip(xvals, yvals)) /
         sum((x - mean_x) * (x - mean_x) for x in xvals))
    a = mean_y - b * mean_x
    return (a, b)
