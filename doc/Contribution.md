# Contributing to the ATF
## Extend the ATF with your own metric
The following steps are needed to implement a new metrics in ATF:
- Create new python-file for the metrics, using the following nameconvention:
```
calculate_*name*.py
```
- copy existing structure from one of the implemented metrics, looking like:
```python
class CalculatePublishRateParamHandler
    def parse_parameter(self, testblock_name, params)	
class CalculatePublishRate:
    def __init__(self, groundtruth, groundtruth_epsilon):
    def start(self, timestamp):
    def stop(self, timestamp):  
    def pause(self, timestamp):
    def purge(self, timestamp):   
    def get_result(self):
```
  using the "publish\_rate"-metrics as an example. Replace "PublishRate" with the name of your newly generated metrics.
- In file ```atf\_metrics\src\atf\_metrics\__init__.py``` add:
```python
from atf_metrics.calculate_*name* import Calculate*Name*, Calculate*Name*ParamHandler
```
  e.g.
```python
from atf_metrics.calculate_jerk import CalculateJerk, CalculateJerkParamHandler
```
  here *name* stands for the name of your new metric (obviously).
  
- In file ```atf\_metrics\config\metrics.yaml``` add:
```
*name*:
   handler: Calculate*Name*ParamHandler
```
  e.g.
```
jerk:
  handler: CalculateJerkParamHandler
```
- In file ```atf\_presenter\html\js\atf\_tools\test\_list.js``` add (using "jerk" as an example):
```javascript
var plot_options = {
      jerk: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Jerk'
        },
        yAxis: {
          title: {
            text: 'Jerk [m/s^3]'
          }
        },
        xAxis: {
          labels: {
            enabled: false
          }
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
};
```
  search the following if-statement:
```javascript
if ((metric_name == 'time') || (metric_name == 'path_length') || (metric_name == 'publish_rate') || (metric_name == 'interface') || (metric_name == 'jerk'))
```
  and add the new metrics as ```|| (metric_name == '*name*')```. In the following lines...
```javascript
if (metric_name == 'path_length') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['root_frame'] + " to " + metric_data['details']['measured_frame'] + ")"
if (metric_name == 'publish_rate') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == 'interface') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
if (metric_name == 'jerk') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```
  add...
```javascript
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```
  to get add information under the metrics-name in the presenter. The "details" you store in the "metrics\_data" will be shown under the metrics-name in brackets.
