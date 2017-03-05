var TestComparison = {
  backup_storage: {},
  categories: {
    speed: ['time'],
    resources: ['resources'],
    efficiency: ['path_length', 'obstacle_distance']
  },
  charts: {
    ids: {},
    data: {},
    invisible: []
  },
  weight: {
    speed: 100,
    resources: 100,
    efficiency: 100
  },
  getMaximum: function (files) {
    //var max = {};
    //$.each(files, function (index, test_name) {
    //  var test_results = FileStorage.readData(test_name);
    //
    //  $.each(test_results, function (testblock_name, testblock_data) {
    //      $.each(testblock_data, function (level_2, level_2_data) {
    //        if (level_2_data.hasOwnProperty('max')) {
    //          // Time
    //          if (!max.hasOwnProperty(level_2)) max[level_2] = 0;
    //          if (level_2_data['max'] > max[level_2]) {
    //            max[level_2] = level_2_data['max'];
    //          }
    //        } else {
    //          $.each(level_2_data, function (level_3, level_3_data) {
    //            if (level_3_data.hasOwnProperty('max')) {
    //              // Path length & obstacle distance
    //              if (!max.hasOwnProperty(level_2)) max[level_2] = 0;
    //              if (level_3_data['max'] > max[level_2]) {
    //                max[level_2] = level_3_data['max'];
    //              }
    //            } else {
    //              $.each(level_3_data, function (level_4, level_4_data) {
    //                // Resources
    //                if (typeof level_4_data['max'][0] === 'undefined') {
    //                  // CPU & Mem
    //                  if (!max.hasOwnProperty(level_4)) max[level_4] = 0;
    //                  if (level_4_data['max'] > max[level_4]) {
    //                    max[level_4] = level_4_data['max'];
    //                  }
    //                } else {
    //                  // IO & Network
    //                  if (!max.hasOwnProperty(level_4)) {
    //                    max[level_4] = [];
    //                    for (var i = 0; i < level_4_data['max'].length; i++) {
    //                      max[level_4].push(0);
    //                    }
    //                  }
    //                  $.each(level_4_data['max'], function (index, value) {
    //                    if (value > max[level_4][index]) {
    //                      max[level_4][index] = value;
    //                    }
    //                  });
    //                }
    //              });
    //            }
    //          });
    //        }
    //      });
    //  });
    //});
    return 10;
  },
  compareTests: function (files) {
    var test_list = FileStorage.readData('test_list');
    this.charts['data'] = {};

    var compare_tests = $('#compare_tests');
    var configuration_div = compare_tests.find('#compare_configuration');
    configuration_div.empty();
    configuration_div.append('<li><b>Scene config: </b>' + test_list[files[0]]['scene_config'] + '</li>' +
      '<li><b>Test config: </b>' + test_list[files[0]]['test_config'] + '</li>' +
      '<li><b>Test repetitions: </b>' + test_list[files[0]]['test_repetitions'] + '</li>');

    var div = $('#test_configuration_details');
    var details_head = div.find('.panel-heading');
    var details_body = div.find('.panel-body');
    details_head.empty();
    details_body.empty();

    var plot_tooltip = {
      formatter: function () {
        if (this.series.name.indexOf('variation') != -1) return false;
        var div = $('#test_configuration_details');
        var details_head = div.find('.panel-heading');
        var details_body = div.find('.panel-body');
        var o = this.point.options;

        details_head.empty();
        details_body.empty();
        details_head.append('Test details: ' + this.series.name);

        $.each(test_list[files[0]], function (name) {
          if (name === 'scene_config' || name === 'test_config' || name === 'test_repetitions' || name === 'subtests') {
            return true;
          }
          details_body.append('<b>' + name.capitalize().replace('_', ' ') + ': </b>' + o[name] + '<br>');
        });

        details_body.append('<b>Average: </b>' + this.y + '%<br>' +
          '<b>Minimum: </b>' + o.min + '%<br>' +
          '<b>Maximum: </b>' + o.max + '%');

        return '<b>' + this.series.name + '</b><br>' +
          'Maximum: ' + o.max + '%<br>' +
          'Average: ' + this.y + '%<br>' +
          'Minimum: ' + o.min + '%<br>';
      }
    };

    this.computePoints(test_list, files);
    this.createCharts(plot_tooltip);
    this.backup_storage = $.extend(true, {}, this.charts['data']);

    this.getBestTests();
  },
  ratingAlgorithm: function (test_config, test_results, max_values) {
    var results = {
      total: {
        average: 0,
        min: 0,
        max: 0
      }
    };

    var categories = [];
    categories.push('total');

    var metrics_in_category = {};

    // Get the metrics needed for each category
    $.each(this.categories, function (category_name, data) {
      for (var x = 0; x < data.length; x++) {
        if (!metrics_in_category.hasOwnProperty(data[x])) {
          metrics_in_category[data[x]] = category_name;
        }
      }
    });

    var temp_testblock = {};
    var this_class = this;
    var data_per_testblock = {};
    var data_per_test = {};
    number_of_testblocks = Object.keys(test_results).length;
    testblock_number = 0;

    // Iterate through all testblocks
    $.each(test_results, function (testblock_name, testblock_data) {
 
      $.each(testblock_data, function (metric_name, metric_list) {
        //console.log("metric_name=", metric_name);
        //console.log("metric_list=", metric_list);
        if ((metric_name == 'time') || (metric_name == 'path_length') || (metric_name == 'publish_rate') || (metric_name == 'interface'))
        {
          number_of_entries = Object.keys(metric_list).length;
          //console.log("number_of_entries=", number_of_entries)
          $.each(metric_list, function(entry_number, metric_data) {
            //console.log("entry_number=", entry_number)
            //console.log("metric_data=", metric_data)
            metric_key_data = metric_data['data'];
            
            // individual settings per metric
            chart_legend_name = testblock_name
            if (metric_name == 'path_length') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['root_frame'] + " to " + metric_data['details']['measured_frame'] + ")"
            if (metric_name == 'publish_rate') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")";
            if (metric_name == 'interface') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")";
            
            if (!data_per_test.hasOwnProperty(metric_name)) data_per_test[metric_name] = [];
            
            if (number_of_testblocks <= 1) {color_testblock = 0}
            else {color_testblock = (testblock_number/(number_of_testblocks-1)*255).round(0)}

            if (number_of_entries <= 1) {color_entry = 0}
            else {color_entry = (entry_number/(number_of_entries-1)*255).round(0)}

            rgb = [255-color_testblock, color_testblock, color_entry]
            color = "rgb(" + rgb[0].toString() + ", " + rgb[1].toString() + ", " + rgb[2].toString() + ")"
            //console.log("color=", color)
            //DATEN SCHREIBEN!!!
            data_per_test[metric_name].push({
              name: chart_legend_name,
              data: [{
                x: testblock_number,
                y: metric_key_data['average'].round(3),
                min: metric_key_data['min'].round(3),
                max: metric_key_data['max'].round(3)
              }],
              color: color
            }, {
              name: chart_legend_name + '_variation',
              type: 'errorbar',
              data: [{
                x: testblock_number,
                low: metric_key_data['min'].round(3),
                high: metric_key_data['max'].round(3)
              },
              { 
                x: testblock_number,
                low: metric_data['groundtruth'] - metric_data['groundtruth_epsilon'],
                high: metric_data['groundtruth'] + metric_data['groundtruth_epsilon'],
                color: "rgb(0, 0, 255)",
                stemWidth: 10,
                stemColor: "rgba(255, 255, 255, 0)",
                whiskerWidth: 3,
                whiskerColor: "rgba(0, 0, 0, 0.5)"
              }]
            });
          })
        }

        if (metric_name == 'resources')
        {
          // Resources
          /*      if (typeof level_4_data['max'][0] === 'undefined') {
                  // CPU & Mem
                  if (!data_per_testblock.hasOwnProperty(level_4)) data_per_testblock[level_4] = [];
                  data_per_testblock[level_4].push({
                    name: level_3,
                    data: [{
                      x: 0,
                      y: level_4_data['average'].round(2),
                      min: level_4_data['min'].round(2),
                      max: level_4_data['max'].round(2)
                    }]
                  }, {
                    name: level_3 + '_variation',
                    type: 'errorbar',
                    data: [{
                      low: level_4_data['min'].round(3),
                      high: level_4_data['max'].round(3)
                    }]
                  });
                } else {
                  var data = [];
                  var data_variation = [];

                  // IO & Network
                  if (!data_per_testblock.hasOwnProperty(level_4)) data_per_testblock[level_4] = [];
                  for (var i = 0; i < level_4_data['min'].length; i++) {
                    if (level_4 === 'io' && i > 1) {
                      level_4_data['average'][i] = (level_4_data['average'][i] / 1000).round(0);
                      level_4_data['min'][i] = (level_4_data['min'][i] / 1000).round(0);
                      level_4_data['max'][i] = (level_4_data['max'][i] / 1000).round(0);
                    } else if (level_4 === 'network' && i < 2) {
                      level_4_data['average'][i] = (level_4_data['average'][i] / 1000).round(0);
                      level_4_data['min'][i] = (level_4_data['min'][i] / 1000).round(0);
                      level_4_data['max'][i] = (level_4_data['max'][i] / 1000).round(0);
                    }
                    data.push({
                      x: i,
                      y: level_4_data['average'][i].round(0),
                      min: level_4_data['min'][i].round(0),
                      max: level_4_data['max'][i].round(0)
                    });
                    data_variation.push({
                      low: level_4_data['min'][i].round(0),
                      high: level_4_data['max'][i].round(0)
                    });
                  }
                  data_per_testblock[level_4].push({
                    name: level_3,
                    data: data
                  }, {
                    name: level_3 + '_variation',
                    type: 'errorbar',
                    data: data_variation
                  });
                }
              });
            }
          });*/
        }
      });
  });
    console.log("data per test", data_per_test);
    console.log("data per testblock", data_per_testblock);

    /*var temp_metrics = {};
    console.log("temp testblock", temp_testblock);
    $.each(data_per_test, function (metric, metric_data) {
      console.log("metric", metric);
      console.log("metric data", metric_data);

      if (!temp_metrics.hasOwnProperty(metric)) {
        temp_metrics[metric] = {
          average: [],
          min: [],
          max: []
        };
      }
      if (!metric_data.hasOwnProperty('average')) {
        $.each(metric_data, function (res_name, res_data) {
          // Resources
          if (res_data['average'].length != 0) {
            temp_metrics[metric]['average'].push(math.mean(res_data['average']));
            temp_metrics[metric]['min'].push(math.mean(res_data['min']));
            temp_metrics[metric]['max'].push(math.mean(res_data['max']));
          }
        });
      } else {
        // Time & Path length & Obstacle distance
        if (metric_data['average'].length != 0) {
          temp_metrics[metric]['average'].push(math.mean(metric_data['average']));
          temp_metrics[metric]['min'].push(math.mean(metric_data['min']));
          temp_metrics[metric]['max'].push(math.mean(metric_data['max']));
        }
      }
    });

    //Get categories from metrics data
    $.each(temp_metrics, function (metric_name) {
      var category = metrics_in_category[metric_name];
      if ($.inArray(category, categories) === -1) {
        categories.push(category);
        results[category] = {
          average: 0,
          min: 0,
          max: 0
        };
      }
    });

    $.each(categories, function (index, category) {
      console.log("category:", category);
      
      if (category != 'total') {
        var temp = {
          average: 0,
          min: 0,
          max: 0
        };
        
        if (category === 'resources') {
          temp['average'] = math.mean(temp_metrics['resources']['average']) * this_class.weight[category];
          temp['min'] = math.mean(temp_metrics['resources']['min']) * this_class.weight[category];
          temp['max'] = math.mean(temp_metrics['resources']['max']) * this_class.weight[category];
        } else if (category === 'speed') {
          temp['average'] = math.mean(temp_metrics['time']['average']) * this_class.weight[category];
          temp['min'] = math.mean(temp_metrics['time']['min']) * this_class.weight[category];
          temp['max'] = math.mean(temp_metrics['time']['max']) * this_class.weight[category];
        } else if (category === 'efficiency') {
          var tp = {
            average: [],
            min: [],
            max: []
          };
          if (temp_metrics.hasOwnProperty('path_length')) {
            tp['average'].push(temp_metrics['path_length']['average']);
            tp['min'].push(temp_metrics['path_length']['min']);
            tp['max'].push(temp_metrics['path_length']['max']);
          }
          if (temp_metrics.hasOwnProperty('obstacle_distance')) {
            tp['average'].push(temp_metrics['obstacle_distance']['average']);
            tp['min'].push(temp_metrics['obstacle_distance']['min']);
            tp['max'].push(temp_metrics['obstacle_distance']['max']);
          }
          temp['average'] = math.mean(tp['average']) * this_class.weight[category];
          temp['min'] = math.mean(tp['min']) * this_class.weight[category];
          temp['max'] = math.mean(tp['max']) * this_class.weight[category];
        }
        results[category]['average'] = temp['average'] * (test_config['test_repetitions'] - test_config['tests_failed']) / test_config['test_repetitions'];
        results[category]['min'] = temp['min'] * (test_config['test_repetitions'] - test_config['tests_failed']) / test_config['test_repetitions'];
        results[category]['max'] = temp['max'] * (test_config['test_repetitions'] - test_config['tests_failed']) / test_config['test_repetitions'];
      }
    });

    var temp = {};
    temp['average'] = [];
    temp['min'] = [];
    temp['max'] = [];

    $.each(categories, function (index, name) {
      if (name != 'total') {
        temp['average'].push(results[name]['average']);
        temp['min'].push(results[name]['min']);
        temp['max'].push(results[name]['max']);
      }
    });*/

    $.each(data_per_testblock, function (metric_name, data) {
        //console.log("metric name", metric_name);
        //console.log("data", data);
        if (data.length != 0) {
          results['total']['min'] = data[metric_name].data.min;
        }
      });


    var details_per_test_panel = $('#details_per_test');
    details_per_test_panel.empty();

    console.log("data_per_test=", data_per_test);
    $.each(data_per_test, function (metric_name, data) {
      console.log("metric name", metric_name);
      console.log("data", data);
      if (data.length != 0){
        console.log("results: ", data[0].data[0].min);
        console.log("results: ", data[0].data[0].max);
        console.log("results: ", data[0].data[0].y);
        results['total']['min'] = data[0].data[0].min;
        results['total']['max'] = data[0].data[0].max;
        results['total']['average'] = data[0].data[0].y;

      }
    });
    
    console.log("results: ", results);
    return results;
  },
  computePoints: function (test_list, files) {
    var this_class = this;
    var max_values = this.getMaximum(files);

    // Iterate through selected tests
    $.each(files, function (index, test_name) {
      var results = this_class.ratingAlgorithm(test_list[test_name], FileStorage.readData("merged_"+test_name), max_values);

      //Save chart data
      $.each(results, function (category, data) {
        if (!this_class.charts['data'].hasOwnProperty(category)) {
          this_class.charts['data'][category] = [];
        }
        this_class.charts['data'][category].push({
          name: test_name,
          data: [{
            x: 0,
            y: results[category]['average'].round(1),
            min: results[category]['min'].round(1),
            max: results[category]['max'].round(1),
            robot: test_list[test_name]['robot'],
            planer_id: test_list[test_name]['planer_id'],
            planning_method: test_list[test_name]['planning_method'],
            jump_threshold: test_list[test_name]['jump_threshold'],
            eef_step: test_list[test_name]['eef_step'],
            tests_failed: test_list[test_name]['tests_failed'] + ' / ' + test_list[test_name]['test_repetitions']
          }]
        }, {
          name: test_name + '_variation',
          type: 'errorbar',
          data: [{
            low: results[category]['min'].round(1),
            high: results[category]['max'].round(1)
          }]
        });
      });
    });
  },
  createCharts: function (plot_tooltip) {
    var this_class = this;

    var category_div = $('#categories_tab');
    var category_tabs = category_div.find('.nav-tabs');
    var category_tabs_content = category_div.find('.tab-content');
    category_tabs.empty();
    category_tabs_content.empty();

    var weight_control_buttons = $('#weight_control').find('.panel-body');
    weight_control_buttons.empty();

    var test_results = $('#test_results').find('.table tbody');
    test_results.empty();

    $('#compare_tab_total').addClass('active');
    $('#compare_tab_categories').removeClass('active');
    $('#total_tab').addClass('active');
    category_div.removeClass('active');

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
      this_class.charts['ids'][category] = new Highcharts.Chart({
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
          },
          series: {
            events: {
              legendItemClick: function () {
                TestComparison.getBestTests(this.name, !this.visible);
              }
            }
          }
        }
      });
    });

    //Create test result table total entry
    test_results.append('<tr><td>Total</td><td id="result_overview_total"></td></tr>');
  },
  changeWeight: function (category, weight) {
    var this_class = this;
    var results_temp = $.extend(true, {}, this.charts['data']);

    var final_results = {
      average: 0,
      min: 0,
      max: 0
    };
    $.each(this.charts['data'][category], function (index, data) {
      if ($.inArray(data.name.split('_variation')[0], this_class.charts['invisible']) === -1) {
        if (data.name.indexOf('variation') === -1) {
          if (data.name)
            if (weight != 0) {
              this_class.backup_storage[category][index]['data'][0]['y'] *= weight;
              this_class.backup_storage[category][index]['data'][0]['min'] *= weight;
              this_class.backup_storage[category][index]['data'][0]['max'] *= weight;
              results_temp = $.extend(true, {}, this_class.backup_storage);
            } else {
              results_temp[category][index]['data'][0]['y'] = 0;
              results_temp[category][index]['data'][0]['max'] = 0;
              results_temp[category][index]['data'][0]['min'] = 0;
            }
          this_class.charts['ids'][category].series[index].setData([{
            y: results_temp[category][index]['data'][0]['y'].round(1),
            min: results_temp[category][index]['data'][0]['min'].round(1),
            max: results_temp[category][index]['data'][0]['max'].round(1)
          }]);

        } else {
          if (weight != 0) {
            this_class.backup_storage[category][index]['data'][0]['low'] *= weight;
            this_class.backup_storage[category][index]['data'][0]['high'] *= weight;
            results_temp = $.extend(true, {}, this_class.backup_storage);
          } else {
            results_temp[category][index]['data'][0]['low'] = 0;
            results_temp[category][index]['data'][0]['high'] = 0;
          }
          this_class.charts['ids'][category].series[index].setData([{
            low: results_temp[category][index]['data'][0]['low'].round(1),
            high: results_temp[category][index]['data'][0]['high'].round(1)
          }]);
        }

        var temp = {
          average: [],
          min: [],
          max: []
        };

        //Calculate final results
        if (data.name.indexOf('variation') === -1) {
          $.each(Object.keys(this_class.charts['data']), function (idx, category_name) {
            if (category_name != 'total') {
              temp['average'].push(results_temp[category_name][index]['data'][0]['y']);
              temp['min'].push(results_temp[category_name][index]['data'][0]['min']);
              temp['max'].push(results_temp[category_name][index]['data'][0]['max']);
            }
          });

          final_results['average'] = math.mean(temp['average']).round(1);
          final_results['min'] = math.mean(temp['min']).round(1);
          final_results['max'] = math.mean(temp['max']).round(1);

          this_class.charts['ids']['total'].series[index].setData([{
            y: final_results['average'],
            min: final_results['min'],
            max: final_results['max']
          }]);
        } else {
          $.each(Object.keys(this_class.charts['data']), function (idx, category_name) {
            if (category_name != 'total') {
              temp['min'].push(results_temp[category_name][index]['data'][0]['low']);
              temp['max'].push(results_temp[category_name][index]['data'][0]['high']);
            }
          });
          final_results['min'] = math.mean(temp['min']).round(1);
          final_results['max'] = math.mean(temp['max']).round(1);

          this_class.charts['ids']['total'].series[index].setData([{
            low: final_results['min'],
            high: final_results['max']
          }]);
        }
      }
    });
    this.getBestTests();
  },
  getBestTests: function (name, visible) {
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
        if (data.name.indexOf('variation') === -1) {
          if (data.name === name && visible === true && $.inArray(name, this_class.charts['invisible']) != -1) {
            this_class.charts['invisible'].splice($.inArray(name, this_class.charts['invisible']), 1);

          } else if (data.name === name && visible === false && $.inArray(name, this_class.charts['invisible']) === -1) {
            this_class.charts['invisible'].push(name);
          }
          if (data['data'][0].y > results[category_name]['value'] && this_class.charts['invisible'].indexOf(data.name) === -1) {
            results[category_name]['value'] = data['data'][0].y;
            results[category_name]['name'] = data.name;
          }
        }
      });
      div.append(results[category_name]['name']);
    });
  }
};
