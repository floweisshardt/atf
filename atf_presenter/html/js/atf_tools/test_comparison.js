var TestComparison = {
  results: {},
  categories: ['total', 'speed', 'resources', 'efficiency'],
  charts: {
    ids: [],
    data: {}
  },
  weightSpeed: 100,
  weightResources: 100,
  weightEfficiency: 100,
  getMaximum: function (files) {
    var max = {};
    $.each(files, function (index, test_name) {
      var test_results = FileStorage.readData(test_name);

      $.each(test_results, function (testblock_name, testblock_data) {
        $.each(testblock_data, function (level_2, level_2_data) {
          if (level_2_data.hasOwnProperty('max')) {
            // Time
            if (!max.hasOwnProperty(level_2)) max[level_2] = 0;
            if (level_2_data['max'] > max[level_2]) {
              max[level_2] = level_2_data['max'];
            }
          } else {
            $.each(level_2_data, function (level_3, level_3_data) {
              if (level_3_data.hasOwnProperty('max')) {
                // Path length & obstacle distance
                if (!max.hasOwnProperty(level_2)) max[level_2] = 0;
                if (level_3_data['max'] > max[level_2]) {
                  max[level_2] = level_3_data['max'];
                }
              } else {
                $.each(level_3_data, function (level_4, level_4_data) {
                  // Resources
                  if (typeof level_4_data['max'][0] === 'undefined') {
                    // CPU & Mem
                    if (!max.hasOwnProperty(level_4)) max[level_4] = 0;
                    if (level_4_data['max'] > max[level_4]) {
                      max[level_4] = level_4_data['max'];
                    }
                  } else {
                    // IO & Network
                    if (!max.hasOwnProperty(level_4)) {
                      max[level_4] = [];
                      for (var i = 0; i < level_4_data['max'].length; i++) {
                        max[level_4].push(0);
                      }
                    }
                    $.each(level_4_data['max'], function (index, value) {
                      if (value > max[level_4][index]) {
                        max[level_4][index] = value;
                      }
                    });
                  }
                });
              }
            });
          }
        });
      });
    });
    return max;
  },
  compare: function (files) {
    var test_list = FileStorage.readData('test_list');

    var compare_tests = $('#compare_tests');
    var configuration_div = compare_tests.find('#compare_configuration');
    configuration_div.empty();
    configuration_div.append('<li><b>Scene config: </b>' + test_list[files[0]]['scene_config'] + '</li>' +
      '<li><b>Test config: </b>' + test_list[files[0]]['test_config'] + '</li>');

    var plot_tooltip = {
      'formatter': function () {
        var div = $('#test_configuration_details');
        var details_head = div.find('.panel-heading');
        var details_body = div.find('.panel-body');
        var o = this.point.options;

        details_head.empty();
        details_body.empty();
        details_head.append('Test details: ' + this.series.name);
        details_body.append('<b>Robot: </b>' + o.robot + '<br>' +
          '<b>Planer ID: </b>' + o.planer + '<br>' +
          '<b>Planning Method: </b>' + o.planning_method + '<br>' +
          '<b>Jump threshold: </b>' + o.jump_threshold + '<br>' +
          '<b>EEF step: </b>' + o.eef_step + '<br><br>' +
          '<b>Average: </b>' + this.y + '%<br>' +
          '<b>Minimum: </b>' + o.min + '%<br>' +
          '<b>Maximum: </b>' + o.max + '%');

        return '<b>' + this.series.name + '</b><br>' +
          'Average: ' + this.y + '<br>' +
          'Minimum: ' + o.min + '<br>' +
          'Maximum: ' + o.max + '<br>';
      }
    };

    this.results = this.computePoints(test_list, files);
    this.charts = this.createCharts(plot_tooltip);

    //this.getBestTests();
  },
  changeWeight: function (category, weight) {
    var final_results = {
      average: []
    };
    var results_temp = jQuery.extend(true, {}, this.results);

    for (var i = 0; i < results_temp[category]['average'].length; i++) {
      results_temp[category]['average'][i] *= weight;
      results_temp[category]['min'][i] *= weight;
      results_temp[category]['max'][i] *= weight;
      this.chart_compare_category_speed.series[i].setData([(results_temp['speed']['average'][i]).round(1)]);
      this.chart_compare_category_efficiency.series[i].setData([(results_temp['efficiency']['average'][i]).round(1)]);
      this.chart_compare_category_resources.series[i].setData([(results_temp['resources']['average'][i]).round(1)]);

      var category_count = 0;
      $.each(results_temp, function (name, value) {
        if (value['average'] != 0) {
          category_count++;
        }
      });

      final_results['average'].push(((results_temp['speed']['average'][i] + results_temp['efficiency']['average'][i] + results_temp['resources']['average'][i]) / category_count).round(1));
      this.chart_compare_overview.series[i].setData([final_results['average'][i]]);
    }
    this.getBestTests();
  },
  getBestTests: function () {
    var results = {
      speed: {
        name: '',
        value: 0
      },
      efficiency: {
        name: '',
        value: 0
      },
      resources: {
        name: '',
        value: 0
      },
      total: {
        name: '',
        value: 0
      }
    };

    var compare_tests = $('#compare_tests').find('#test_results');
    var speed = compare_tests.find('#result_overview_speed');
    var efficiency = compare_tests.find('#result_overview_efficiency');
    var resources = compare_tests.find('#result_overview_resources');
    var total = compare_tests.find('#result_overview_total');

    $.each(this.chart_compare_category_speed.series, function (index, data) {
      if (data['data'][0].y > results['speed']['value']) {
        results['speed']['value'] = data['data'][0].y;
        results['speed']['name'] = data.name;
      }
    });
    $.each(this.chart_compare_category_efficiency.series, function (index, data) {
      if (data['data'][0].y > results['efficiency']['value']) {
        results['efficiency']['value'] = data['data'][0].y;
        results['efficiency']['name'] = data.name;
      }
    });
    $.each(this.chart_compare_category_resources.series, function (index, data) {
      if (data['data'][0].y > results['resources']['value']) {
        results['resources']['value'] = data['data'][0].y;
        results['resources']['name'] = data.name;
      }
    });
    $.each(this.chart_compare_overview.series, function (index, data) {
      if (data['data'][0].y > results['total']['value']) {
        results['total']['value'] = data['data'][0].y;
        results['total']['name'] = data.name;
      }
    });

    speed.empty();
    efficiency.empty();
    resources.empty();
    total.empty();

    speed.append(results['speed']['name']);
    efficiency.append(results['efficiency']['name']);
    resources.append(results['resources']['name']);
    total.append(results['total']['name']);
  },
  computePoints: function (test_list, files) {
    var this_class = this;
    var results = {};
    var max_values = this.getMaximum(files);

    $.each(this.categories, function (index, category_name) {
      results[category_name] = {
        min: [],
        max: [],
        average: []
      };
    });

    // Iterate through selected tests
    $.each(files, function (index, test_name) {
      var test_results = FileStorage.readData(test_name);

      var temp_testblock = {};
      var temp_metrics = {};
      var count_resource_categories = 0;

      $.each(TestList.metrics, function (metric_name, metric_data) {
        if (!temp_metrics.hasOwnProperty(metric_name)) {
          temp_metrics[metric_name] = {
            min: 0,
            max: 0,
            average: 0
          };
        }
        if (metric_data.length === 0) {
          temp_testblock[metric_name] = {
            length: 0,
            min: 0,
            max: 0,
            average: 0,
            total: max_values[metric_name]
          };
        } else {
          temp_testblock[metric_name] = {};
          $.each(metric_data, function (index, sub_metric_name) {
            temp_testblock[metric_name][sub_metric_name] = {
              length: 0,
              min: 0,
              max: 0,
              average: 0,
              total: max_values[sub_metric_name]
            };
          });
        }
      });

      // Iterate through all testblocks
      $.each(test_results, function (testblock_name, testblock_data) {
        $.each(testblock_data, function (level_2, level_2_data) {
          if (level_2_data.hasOwnProperty('max')) {
            // Time
            if (temp_testblock[level_2]['total'] != 0) {
              temp_testblock[level_2]['average'] += (temp_testblock[level_2]['total'] - level_2_data['average']) / temp_testblock[level_2]['total'];
              temp_testblock[level_2]['min'] += (temp_testblock[level_2]['total'] - level_2_data['min']) / temp_testblock[level_2]['total'];
              temp_testblock[level_2]['max'] += (temp_testblock[level_2]['total'] - level_2_data['max']) / temp_testblock[level_2]['total'];
            }
            temp_testblock[level_2]['length']++;
          } else {
            $.each(level_2_data, function (level_3, level_3_data) {
              if (level_3_data.hasOwnProperty('max')) {
                // Path length & obstacle distance
                if (temp_testblock[level_2]['total'] != 0) {
                  if (level_2 === 'obstacle_distance') {
                    temp_testblock[level_2]['average'] += level_3_data['average'] / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['min'] += level_3_data['min'] / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['max'] += level_3_data['max'] / temp_testblock[level_2]['total'];
                  } else {
                    temp_testblock[level_2]['average'] += (temp_testblock[level_2]['total'] - level_3_data['average']) / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['min'] += (temp_testblock[level_2]['total'] - level_3_data['min']) / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['max'] += (temp_testblock[level_2]['total'] - level_3_data['max']) / temp_testblock[level_2]['total'];
                  }
                }
                temp_testblock[level_2]['length']++;
              } else {
                $.each(level_3_data, function (level_4, level_4_data) {
                  // Resources
                  if (typeof level_4_data['max'][0] === 'undefined') {
                    // CPU & Mem
                    if (temp_testblock['resources'][level_4]['total'] != 0) {
                      temp_testblock['resources'][level_4]['average'] += (temp_testblock['resources'][level_4]['total'] - level_4_data['average']) / temp_testblock['resources'][level_4]['total'];
                      temp_testblock['resources'][level_4]['min'] += (temp_testblock['resources'][level_4]['total'] - level_4_data['min']) / temp_testblock['resources'][level_4]['total'];
                      temp_testblock['resources'][level_4]['max'] += (temp_testblock['resources'][level_4]['total'] - level_4_data['max']) / temp_testblock['resources'][level_4]['total'];
                    }
                    temp_testblock['resources'][level_4]['length']++;
                  } else {
                    // IO & Network
                    for (var x = 0; x < level_4_data.length; x++) {
                      if (temp_testblock['resources'][level_4]['total'][x] != 0) {
                        temp_testblock['resources'][level_4]['average'] += (temp_testblock['resources'][level_4]['total'][x] - level_4_data['average'][x]) / temp_testblock['resources'][level_4]['total'][x];
                        temp_testblock['resources'][level_4]['min'] += (temp_testblock['resources'][level_4]['total'][x] - level_4_data['min'][x]) / temp_testblock['resources'][level_4]['total'][x];
                        temp_testblock['resources'][level_4]['max'] += (temp_testblock['resources'][level_4]['total'][x] - level_4_data['max'][x]) / temp_testblock['resources'][level_4]['total'][x];
                      }
                      temp_testblock['resources'][level_4]['length']++;
                    }
                  }
                });
              }
            });
          }
        });
      });
      //TODO: Remove real metric names
      $.each(temp_testblock, function (metric, metric_data) {
        if (metric === 'resources') {
          $.each(metric_data, function (res_name, res_data) {
            // Resources
            if (res_data['length'] != 0) {
              temp_metrics[metric]['average'] += res_data['average'] / res_data['length'];
              temp_metrics[metric]['min'] += res_data['min'] / res_data['length'];
              temp_metrics[metric]['max'] += res_data['max'] / res_data['length'];
              count_resource_categories++;
            }
          });
        } else {
          // Time & Path length & Obstacle distance
          if (metric_data['length'] != 0) {
            temp_metrics[metric]['average'] = metric_data['average'] / metric_data['length'];
            temp_metrics[metric]['min'] = metric_data['min'] / metric_data['length'];
            temp_metrics[metric]['max'] = metric_data['max'] / metric_data['length'];
          }
        }
      });

      results['resources']['average'].push(temp_metrics['resources']['average'] / count_resource_categories * this_class.weightResources);
      results['resources']['min'].push(temp_metrics['resources']['min'] / count_resource_categories * this_class.weightResources);
      results['resources']['max'].push(temp_metrics['resources']['max'] / count_resource_categories * this_class.weightResources);

      results['speed']['average'].push(temp_metrics['time']['average'] * this_class.weightSpeed);
      results['speed']['min'].push(temp_metrics['time']['min'] * this_class.weightSpeed);
      results['speed']['max'].push(temp_metrics['time']['max'] * this_class.weightSpeed);

      if (temp_metrics['path_length']['average'] != 0 && temp_metrics['obstacle_distance']['average'] != 0) {
        results['efficiency']['average'].push(((temp_metrics['path_length']['average'] + temp_metrics['obstacle_distance']['average']) / 2) * this_class.weightEfficiency);
        results['efficiency']['min'].push(((temp_metrics['path_length']['min'] + temp_metrics['obstacle_distance']['min']) / 2) * this_class.weightEfficiency);
        results['efficiency']['max'].push(((temp_metrics['path_length']['max'] + temp_metrics['obstacle_distance']['max']) / 2) * this_class.weightEfficiency);
      } else {
        results['efficiency']['average'].push((temp_metrics['path_length']['average'] + temp_metrics['obstacle_distance']['average']) * this_class.weightEfficiency);
        results['efficiency']['min'].push((temp_metrics['path_length']['min'] + temp_metrics['obstacle_distance']['min']) * this_class.weightEfficiency);
        results['efficiency']['max'].push((temp_metrics['path_length']['max'] + temp_metrics['obstacle_distance']['max']) * this_class.weightEfficiency);
      }

      var category_count = 0;
      $.each(results, function (name, value) {
        if (value['average'] != 0 && name != 'total') {
          category_count++;
        }
      });

      results['total']['average'].push((results['speed']['average'][results['speed']['average'].length - 1] +
        results['efficiency']['average'][results['efficiency']['average'].length - 1] +
        results['resources']['average'][results['resources']['average'].length - 1]) / category_count);
      results['total']['min'].push((results['speed']['min'][results['speed']['min'].length - 1] +
        results['efficiency']['min'][results['efficiency']['min'].length - 1] +
        results['resources']['min'][results['resources']['min'].length - 1]) / category_count);
      results['total']['max'].push((results['speed']['max'][results['speed']['max'].length - 1] +
        results['efficiency']['max'][results['efficiency']['max'].length - 1] +
        results['resources']['max'][results['resources']['max'].length - 1]) / category_count);

      //Save chart data
      $.each(results, function (category, data) {
        if (!this_class.charts['data'].hasOwnProperty(category)) {
          this_class.charts['data'][category] = [];
        }
        this_class.charts['data'][category].push({
          name: test_name,
          data: [{
            x: 0,
            y: results[category]['average'][results[category]['average'].length - 1].round(1),
            min: results[category]['min'][results[category]['min'].length - 1].round(1),
            max: results[category]['max'][results[category]['max'].length - 1].round(1),
            robot: test_list[test_name]['robot'],
            planer: test_list[test_name]['planer_id'],
            planning_method: test_list[test_name]['planning_method'],
            jump_threshold: test_list[test_name]['jump_threshold'],
            eef_step: test_list[test_name]['eef_step']
          }]
        });
      });

    });
    return results;
  },
  createCharts: function (plot_tooltip) {
    var charts = {};
    var category_div = $('#categories_tab');
    var category_tabs = category_div.find('.nav-tabs');
    var category_tabs_content = category_div.find('.tab-content');
    category_tabs.empty();
    category_tabs_content.empty();
    var active = true;
    var class_active = '';

    $.each(this.charts['data'], function (category, data) {
      if (!charts.hasOwnProperty('ids')) {
        charts['ids'] = [];
      }
      if (category != 'total') {
        if (active) {
          class_active = 'active';
          active = false;
        } else {
          class_active = '';
        }
        //TODO: Capitalize first letter of category
        //TODO: Add weight buttons & result overview div
        category_tabs.append('<li role="presentation" class="' + class_active + '"><a href="#' + category + '_tab"' +
          'aria-controls="' + category + '_tab" role="tab" data-toggle="tab">' + category + '</a></li>');
        category_tabs_content.append('<div role="tabpanel" class="tab-pane ' + class_active + '" id="' + category + '_tab">' +
          '<div id="' + category + '" class="plot"></div></div>');
      }

      charts['ids'].push(new Highcharts.Chart({
        chart: {
          renderTo: category,
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: ''
        },
        xAxis: {
          labels: {
            enabled: false
          },
          minTickInterval: 0
        },
        yAxis: {
          title: {
            text: 'Percentage [%]'
          }
        },
        tooltip: plot_tooltip,
        series: data,
        plotOptions: {
          column: {
            pointPadding: 0,
            groupPadding: 0,
            borderWidth: 0
          }
        }
      }));
    });
    return charts;
  }
};
