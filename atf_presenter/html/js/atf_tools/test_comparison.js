var TestComparison = {
  results: {},
  categories: {
    speed: ['time'],
    resources: ['resources'],
    efficiency: ['path_length', 'obstacle_distance']
  },
  charts: {
    ids: {},
    data: {}
  },
  weight: {
    speed: 100,
    resources: 100,
    efficiency: 100
  },
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
    this.charts['data'] = {};

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

    this.getBestTests();
  },
  changeWeight: function (category, weight) {
    var results_temp = $.extend(true, {}, this.results);
    var final_results = {
      average: [],
      min: [],
      max: []
    };

    for (var i = 0; i < this.results[category]['average'].length; i++) {
      if (weight != 0) {
        this.results[category]['average'][i] *= weight;
        this.results[category]['min'][i] *= weight;
        this.results[category]['max'][i] *= weight;
        results_temp = $.extend(true, {}, this.results);
      } else {
        results_temp[category]['average'][i] = 0;
        results_temp[category]['min'][i] = 0;
        results_temp[category]['max'][i] = 0;
      }

      this.charts['ids'][category].series[i].setData([{
        y: results_temp[category]['average'][i].round(1),
        min: results_temp[category]['min'][i].round(1),
        max: results_temp[category]['max'][i].round(1)
      }]);

      var temp = {};
      temp['average'] = 0;
      temp['min'] = 0;
      temp['max'] = 0;
      $.each(Object.keys(this.charts['data']), function (index, category_name) {

        if (category_name != 'total') {
          temp['average'] += results_temp[category_name]['average'][i];
          temp['min'] += results_temp[category_name]['min'][i];
          temp['max'] += results_temp[category_name]['max'][i];
        }
      });

      final_results['average'].push((temp['average'] / (Object.keys(this.charts['data']).length - 1)).round(1));
      final_results['min'].push((temp['min'] / (Object.keys(this.charts['data']).length - 1)).round(1));
      final_results['max'].push((temp['max'] / (Object.keys(this.charts['data']).length - 1)).round(1));

      this.charts['ids']['total'].series[i].setData([{
        y: final_results['average'][i],
        min: final_results['min'][i],
        max: final_results['max'][i]
      }]);
    }
    this.getBestTests();
  },
  getBestTests: function () {
    var results = {};
    var compare_tests = $('#compare_tests').find('#test_results');
    var this_class = this;

    $.each(Object.keys(this.charts['data']), function (index, category_name) {

      if (!results.hasOwnProperty(category_name)) {
        results[category_name] = {
          name: '',
          value: 0
        };
      }
      var div = compare_tests.find('#result_overview_' + category_name);
      div.empty();

      $.each(this_class.charts['ids'][category_name].series, function (index, data) {
        if (data['data'][0].y > results[category_name]['value']) {
          results[category_name]['value'] = data['data'][0].y;
          results[category_name]['name'] = data.name;
        }
      });
      div.append(results[category_name]['name']);
    });
  },
  computePoints: function (test_list, files) {
    var this_class = this;
    var results = {};
    results['total'] = {
      average: [],
      min: [],
      max: []
    };
    var categories = [];
    categories.push('total');
    var max_values = this.getMaximum(files);
    var metrics_to_category = {};

    // Get the metrics needed for each category
    $.each(this.categories, function (category_name, data) {
      for (var x = 0; x < data.length; x++) {
        if (!metrics_to_category.hasOwnProperty(data[x])) {
          metrics_to_category[data[x]] = category_name;
        }
      }
    });

    // Iterate through selected tests
    $.each(files, function (index, test_name) {
      var test_results = FileStorage.readData(test_name);

      var temp_testblock = {};

      // Iterate through all testblocks
      $.each(test_results, function (testblock_name, testblock_data) {
        $.each(testblock_data, function (level_2, level_2_data) {
          if (level_2_data.hasOwnProperty('max')) {
            // Time
            if (!temp_testblock.hasOwnProperty(level_2)) {
              temp_testblock[level_2] = {
                length: 0,
                total: max_values[level_2],
                min: 0,
                max: 0,
                average: 0
              };
            }
            if (temp_testblock[level_2]['total'] != 0) {
              temp_testblock[level_2]['average'] += (temp_testblock[level_2]['total'] - level_2_data['average']) / temp_testblock[level_2]['total'];
              temp_testblock[level_2]['max'] += (temp_testblock[level_2]['total'] - level_2_data['min']) / temp_testblock[level_2]['total'];
              temp_testblock[level_2]['min'] += (temp_testblock[level_2]['total'] - level_2_data['max']) / temp_testblock[level_2]['total'];
            }
            temp_testblock[level_2]['length']++;
          } else {
            $.each(level_2_data, function (level_3, level_3_data) {
              if (level_3_data.hasOwnProperty('max')) {
                // Path length & obstacle distance
                if (!temp_testblock.hasOwnProperty(level_2)) {
                  temp_testblock[level_2] = {
                    length: 0,
                    total: max_values[level_2],
                    min: 0,
                    max: 0,
                    average: 0
                  };
                }
                if (temp_testblock[level_2]['total'] != 0) {
                  if (level_2 === 'obstacle_distance') {
                    temp_testblock[level_2]['average'] += level_3_data['average'] / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['min'] += level_3_data['min'] / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['max'] += level_3_data['max'] / temp_testblock[level_2]['total'];
                  } else {
                    temp_testblock[level_2]['average'] += (temp_testblock[level_2]['total'] - level_3_data['average']) / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['max'] += (temp_testblock[level_2]['total'] - level_3_data['min']) / temp_testblock[level_2]['total'];
                    temp_testblock[level_2]['min'] += (temp_testblock[level_2]['total'] - level_3_data['max']) / temp_testblock[level_2]['total'];
                  }
                }
                temp_testblock[level_2]['length']++;
              } else {
                $.each(level_3_data, function (level_4, level_4_data) {
                  // Resources
                  if (!temp_testblock.hasOwnProperty(level_2)) {
                    temp_testblock[level_2] = {};
                  }
                  if (typeof level_4_data['max'][0] === 'undefined') {
                    // CPU & Mem
                    if (!temp_testblock[level_2].hasOwnProperty(level_4)) {
                      temp_testblock[level_2][level_4] = {
                        length: 0,
                        min: 0,
                        max: 0,
                        average: 0,
                        total: max_values[level_4]
                      };
                    }
                    if (temp_testblock[level_2][level_4]['total'] != 0) {
                      temp_testblock[level_2][level_4]['average'] += (temp_testblock[level_2][level_4]['total'] - level_4_data['average']) / temp_testblock[level_2][level_4]['total'];
                      temp_testblock[level_2][level_4]['max'] += (temp_testblock[level_2][level_4]['total'] - level_4_data['min']) / temp_testblock[level_2][level_4]['total'];
                      temp_testblock[level_2][level_4]['min'] += (temp_testblock[level_2][level_4]['total'] - level_4_data['max']) / temp_testblock[level_2][level_4]['total'];
                    }
                    temp_testblock[level_2][level_4]['length']++;
                  } else {
                    // IO & Network
                    if (!temp_testblock[level_2].hasOwnProperty(level_4)) {
                      temp_testblock[level_2][level_4] = {
                        length: 0,
                        min: 0,
                        max: 0,
                        average: 0,
                        total: max_values[level_4]
                      };
                    }
                    for (var x = 0; x < level_4_data['max'].length; x++) {
                      if (temp_testblock[level_2][level_4]['total'][x] != 0) {
                        temp_testblock[level_2][level_4]['average'] += (temp_testblock[level_2][level_4]['total'][x] - level_4_data['average'][x]) / temp_testblock[level_2][level_4]['total'][x];
                        temp_testblock[level_2][level_4]['max'] += (temp_testblock[level_2][level_4]['total'][x] - level_4_data['min'][x]) / temp_testblock[level_2][level_4]['total'][x];
                        temp_testblock[level_2][level_4]['min'] += (temp_testblock[level_2][level_4]['total'][x] - level_4_data['max'][x]) / temp_testblock[level_2][level_4]['total'][x];
                      }
                      temp_testblock[level_2][level_4]['length']++;
                    }
                  }
                });
              }
            });
          }
        });
      });

      var temp_metrics = {};
      var count_resource_categories = 0;

      $.each(temp_testblock, function (metric, metric_data) {
        if (!temp_metrics.hasOwnProperty(metric)) {
          temp_metrics[metric] = {
            average: 0,
            min: 0,
            max: 0
          };
        }
        if (!metric_data.hasOwnProperty('average')) {
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

      //Get categories from metrics data
      $.each(temp_metrics, function (metric_name) {
        var category = metrics_to_category[metric_name];
        if ($.inArray(category, categories) === -1) {
          categories.push(category);
          results[category] = {
            average: [],
            min: [],
            max: []
          };
        }
      });

      $.each(categories, function (index, category) {
        if (category != 'total') {
          var temp = {
            average: 0,
            min: 0,
            max: 0
          };

          if (category === 'resources') {
            temp['average'] = (temp_metrics['resources']['average'] / count_resource_categories) * this_class.weight[category];
            temp['min'] = (temp_metrics['resources']['min'] / count_resource_categories) * this_class.weight[category];
            temp['max'] = (temp_metrics['resources']['max'] / count_resource_categories) * this_class.weight[category];
          } else if (category === 'speed') {
            temp['average'] = temp_metrics['time']['average'] * this_class.weight[category];
            temp['min'] = temp_metrics['time']['min'] * this_class.weight[category];
            temp['max'] = temp_metrics['time']['max'] * this_class.weight[category];
          } else if (category === 'efficiency') {
            if (temp_metrics['path_length']['average'] != 0 && temp_metrics['obstacle_distance']['average'] != 0) {
              temp['average'] = ((temp_metrics['path_length']['average'] + temp_metrics['obstacle_distance']['average']) / 2) * this_class.weight[category];
              temp['min'] = ((temp_metrics['path_length']['min'] + temp_metrics['obstacle_distance']['min']) / 2) * this_class.weight[category];
              temp['max'] = ((temp_metrics['path_length']['max'] + temp_metrics['obstacle_distance']['max']) / 2) * this_class.weight[category];
            } else {
              temp['average'] = (temp_metrics['path_length']['average'] + temp_metrics['obstacle_distance']['average']) * this_class.weight[category];
              temp['min'] = (temp_metrics['path_length']['min'] + temp_metrics['obstacle_distance']['min']) * this_class.weight[category];
              temp['max'] = (temp_metrics['path_length']['max'] + temp_metrics['obstacle_distance']['max']) * this_class.weight[category];
            }
          }
          results[category]['average'].push(temp['average']);
          results[category]['min'].push(temp['min']);
          results[category]['max'].push(temp['max']);
        }
      });

      var temp = {};
      temp['average'] = 0;
      temp['min'] = 0;
      temp['max'] = 0;

      $.each(categories, function (index, name) {
        if (name != 'total') {
          temp['average'] += results[name]['average'][results[name]['average'].length - 1];
          temp['min'] += results[name]['min'][results[name]['min'].length - 1];
          temp['max'] += results[name]['max'][results[name]['max'].length - 1];
        }
      });

      results['total']['average'].push(temp['average'] / (categories.length - 1));
      results['total']['min'].push(temp['min'] / (categories.length - 1));
      results['total']['max'].push(temp['max'] / (categories.length - 1));

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
    charts['ids'] = {};
    charts['data'] = this.charts['data'];
    
    var category_div = $('#categories_tab');
    var category_tabs = category_div.find('.nav-tabs');
    var category_tabs_content = category_div.find('.tab-content');
    category_tabs.empty();
    category_tabs_content.empty();
    
    var weight_control_buttons = $('#weight_control').find('.panel-body');
    weight_control_buttons.empty();

    var test_results = $('#test_results').find('.table tbody');
    test_results.empty();
    
    var active = true;
    var class_active = '';

    $.each(this.charts['data'], function (category, data) {
      if (category != 'total') {
        if (active) {
          class_active = 'active';
          active = false;
        } else {
          class_active = '';
        }
        //Create category tab
        category_tabs.append('<li role="presentation" class="' + class_active + '"><a href="#' + category + '_tab"' +
          'aria-controls="' + category + '_tab" role="tab" data-toggle="tab">' + category.capitalize() + '</a></li>');
        category_tabs_content.append('<div role="tabpanel" class="tab-pane ' + class_active + '" id="' + category + '_tab">' +
          '<div id="' + category + '" class="plot"></div></div>');
        
        //Create weight control button
        weight_control_buttons.append('<div class="input-group"><span class="input-group-btn">' +
          '<button type="button" class="btn btn-primary weight_control_button" value="' + category + '">' + category.capitalize() + '</button>' +
          '</span><input type="text" class="form-control weight_control_value" placeholder="' + category.capitalize() + ' weight value" value="1.5">' +
          '</div><div class="placeholder"></div>');

        //Create test result table entries
        test_results.append('<tr><td>' + category.capitalize() + '</td><td id="result_overview_' + category + '"></td></tr>');
      }
      charts['ids'][category] = new Highcharts.Chart({
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
      });
    });

    //Create test result table total entry
    test_results.append('<tr><td>Total</td><td id="result_overview_total"></td></tr>');

    return charts;
  }
};
