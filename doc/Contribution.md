# Contributing to the ATF
Auto-generated TOC with https://imthenachoman.github.io/nGitHubTOC/.
- [Extend the ATF with your own metric](#extend-the-atf-with-your-own-metric)
  - [Python File](#python-file)

## Extend the ATF with your own metric
The following steps are needed to implement a new metrics in ATF:
### Python File
- Create new python-file for the metrics, using the following nameconvention:
```
calculate_*name*.py
```
- copy existing structure from one of the implemented metrics, looking like:
```python
class CalculatePublishRateParamHandler
    def parse_parameter(self, testblock_name, params):
class CalculatePublishRate:
    def __init__(self, groundtruth, groundtruth_epsilon):
    def start(self, timestamp):
    def stop(self, timestamp):
    def pause(self, timestamp):
    def purge(self, timestamp):
    def get_result(self):
```
  using the "publish\_rate"-metrics as an example. Replace "PublishRate" with the name of your newly generated metrics.
- In file ```atf/src/atf/atf_metrics/src/atf_metrics/__init__.py``` add:
```python
from atf_metrics.calculate_*name* import Calculate*Name*, Calculate*Name*ParamHandler
```
  e.g.
```python
from atf_metrics.calculate_jerk import CalculateJerk, CalculateJerkParamHandler
```
  here *name* stands for the name of your new metric (obviously).
  
- In file ```atf/src/atf/atf_metrics/config/metrics.yaml``` add:
```
*name*:
   handler: Calculate*Name*ParamHandler
```
  e.g.
```
jerk:
  handler: CalculateJerkParamHandler
```
