var chart_compare_overview;
var chart_compare_category_speed;
var chart_compare_category_efficiency;
var chart_compare_category_resources;

function searchForMaximum(tests) {
  var max = {
    cpu: 0,
    mem: 0,
    io: [0, 0, 0, 0],
    network: [0, 0, 0, 0, 0, 0, 0, 0],
    time: 0,
    path_length: 0,
    obstacle_distance: 0
  };
  $.each(tests, function (index, test_name) {
    FileStorage.name = test_name;
    var test_results = FileStorage.readData();

    $.each(test_results, function (testblock_name, testblock_data) {
      $.each(testblock_data, function (level_2, level_2_data) {
        if (level_2_data.hasOwnProperty('max')) {
          // Time
          if (level_2_data['max'] > max[level_2]) {
            max[level_2] = level_2_data['max'];
          }
        } else {
          $.each(level_2_data, function (level_3, level_3_data) {
            if (level_3_data.hasOwnProperty('max')) {
              // Path length & obstacle distance
              if (level_3_data['max'] > max[level_2]) {
                max[level_2] = level_3_data['max'];
              }
            } else {
              $.each(level_3_data, function (level_4, level_4_data) {
                // Resources
                if (typeof level_4_data['max'][0] === 'undefined') {
                  // CPU & Mem
                  if (level_4_data['max'] > max[level_4]) {
                    max[level_4] = level_4_data['max'];
                  }
                } else {
                  // IO & Network
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
}

function compareTests(tests) {
  var compare_tests = $('#compare_tests');
  var configuration_div = compare_tests.find('#compare_configuration');

  FileStorage.name = 'test_list';
  var test_list = FileStorage.readData();

  var data_overview = [];
  var data_categories = {
    speed: [],
    efficiency: [],
    resources: []
  };

  results = {
    speed: {
      min: [],
      max: [],
      average: []
    },
    resources: {
      min: [],
      max: [],
      average: []
    },
    efficiency: {
      min: [],
      max: [],
      average: []
    }
  };

  var WEIGHT_SPEED = 100;
  var WEIGHT_RESOURCES = 100;
  var WEIGHT_EFFICIENCY = 100;

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

  configuration_div.empty();
  configuration_div.append('<li><b>Scene config: </b>' + test_list[tests[0]]['scene_config'] + '</li>' +
    '<li><b>Test config: </b>' + test_list[tests[0]]['test_config'] + '</li>');

  var max_values = searchForMaximum(tests);

  $.each(tests, function (index, test_name) {

    var category_results = {
      speed: {
        min: 0,
        max: 0,
        average: 0
      },
      efficiency: {
        min: 0,
        max: 0,
        average: 0
      },
      resources: {
        min: 0,
        max: 0,
        average: 0
      }
    };

    FileStorage.name = test_name;
    var test_results = FileStorage.readData();
    var temp_testblock = {
      time: {
        length: 0,
        min: 0,
        max: 0,
        average: 0,
        total: max_values['time']
      },
      path_length: {
        length: 0,
        min: 0,
        max: 0,
        average: 0,
        total: max_values['path_length']
      },
      obstacle_distance: {
        length: 0,
        min: 0,
        max: 0,
        average: 0,
        total: max_values['obstacle_distance']
      },
      resources: {
        cpu: {
          length: 0,
          min: 0,
          max: 0,
          average: 0,
          total: max_values['cpu']
        },
        mem: {
          length: 0,
          min: 0,
          max: 0,
          average: 0,
          total: max_values['mem']
        },
        io: {
          length: 0,
          min: 0,
          max: 0,
          average: 0,
          total: max_values['io']
        },
        network: {
          length: 0,
          min: 0,
          max: 0,
          average: 0,
          total: max_values['network']
        }
      }
    };

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

    var temp_categories = {
      resources: {
        min: 0,
        max: 0,
        average: 0
      },
      time: {
        min: 0,
        max: 0,
        average: 0
      },
      path_length: {
        min: 0,
        max: 0,
        average: 0
      },
      obstacle_distance: {
        min: 0,
        max: 0,
        average: 0
      }
    };

    var count_resource_categories = 0;

    $.each(temp_testblock, function (metric, metric_data) {
      if (metric === 'resources') {
        $.each(metric_data, function (res_name, res_data) {
          // Resources
          if (res_data['length'] != 0) {
            temp_categories[metric]['average'] += res_data['average'] / res_data['length'];
            temp_categories[metric]['min'] += res_data['min'] / res_data['length'];
            temp_categories[metric]['max'] += res_data['max'] / res_data['length'];
            count_resource_categories++;
          }
        });
      } else {
        // Time & Path length & Obstacle distance
        if (metric_data['length'] != 0) {
          temp_categories[metric]['average'] = metric_data['average'] / metric_data['length'];
          temp_categories[metric]['min'] = metric_data['min'] / metric_data['length'];
          temp_categories[metric]['max'] = metric_data['max'] / metric_data['length'];
        }
      }
    });
    var temp_resources = {};
    var temp_speed = {};
    var temp_efficiency = {};

    temp_resources['average'] = temp_categories['resources']['average'] / count_resource_categories;
    temp_resources['min'] = temp_categories['resources']['min'] / count_resource_categories;
    temp_resources['max'] = temp_categories['resources']['max'] / count_resource_categories;

    temp_speed['average'] = temp_categories['time']['average'];
    temp_speed['min'] = temp_categories['time']['min'];
    temp_speed['max'] = temp_categories['time']['max'];

    if (temp_categories['path_length']['average'] != 0 && temp_categories['obstacle_distance']['average'] != 0) {
      temp_efficiency['average'] = (temp_categories['path_length']['average'] + temp_categories['obstacle_distance']['average']) / 2;
      temp_efficiency['min'] = (temp_categories['path_length']['min'] + temp_categories['obstacle_distance']['min']) / 2;
      temp_efficiency['max'] = (temp_categories['path_length']['max'] + temp_categories['obstacle_distance']['max']) / 2;
    } else {
      temp_efficiency['average'] = temp_categories['path_length']['average'] + temp_categories['obstacle_distance']['average'];
      temp_efficiency['min'] = temp_categories['path_length']['min'] + temp_categories['obstacle_distance']['min'];
      temp_efficiency['max'] = temp_categories['path_length']['max'] + temp_categories['obstacle_distance']['max'];
    }

    results['speed']['average'].push(temp_speed['average'] * WEIGHT_SPEED);
    results['speed']['min'].push(temp_speed['min'] * WEIGHT_SPEED);
    results['speed']['max'].push(temp_speed['max'] * WEIGHT_SPEED);

    results['efficiency']['average'].push(temp_efficiency['average'] * WEIGHT_EFFICIENCY);
    results['efficiency']['min'].push(temp_efficiency['min'] * WEIGHT_EFFICIENCY);
    results['efficiency']['max'].push(temp_efficiency['max'] * WEIGHT_EFFICIENCY);

    results['resources']['average'].push(temp_resources['average'] * WEIGHT_RESOURCES);
    results['resources']['min'].push(temp_resources['min'] * WEIGHT_RESOURCES);
    results['resources']['max'].push(temp_resources['max'] * WEIGHT_RESOURCES);

    category_results['speed']['average'] = (temp_speed['average'] * WEIGHT_SPEED).round(1);
    category_results['speed']['min'] = (temp_speed['min'] * WEIGHT_SPEED).round(1);
    category_results['speed']['max'] = (temp_speed['max'] * WEIGHT_SPEED).round(1);

    category_results['efficiency']['average'] = (temp_efficiency['average'] * WEIGHT_EFFICIENCY).round(1);
    category_results['efficiency']['min'] = (temp_efficiency['min'] * WEIGHT_EFFICIENCY).round(1);
    category_results['efficiency']['max'] = (temp_efficiency['max'] * WEIGHT_EFFICIENCY).round(1);

    category_results['resources']['average'] = (temp_resources['average'] * WEIGHT_RESOURCES).round(1);
    category_results['resources']['min'] = (temp_resources['min'] * WEIGHT_RESOURCES).round(1);
    category_results['resources']['max'] = (temp_resources['max'] * WEIGHT_RESOURCES).round(1);

    var category_count = 0;
    $.each(category_results, function (name, value) {
      if (value['average'] != 0) {
        category_count++;
      }
    });
    var final_results = {};
    final_results['average'] = ((temp_speed['average'] * WEIGHT_SPEED + temp_efficiency['average'] * WEIGHT_EFFICIENCY + temp_resources['average'] * WEIGHT_RESOURCES) / category_count).round(1);
    final_results['min'] = ((temp_speed['min'] * WEIGHT_SPEED + temp_efficiency['min'] * WEIGHT_EFFICIENCY + temp_resources['min'] * WEIGHT_RESOURCES) / category_count).round(1);
    final_results['max'] = ((temp_speed['max'] * WEIGHT_SPEED + temp_efficiency['max'] * WEIGHT_EFFICIENCY + temp_resources['max'] * WEIGHT_RESOURCES) / category_count).round(1);

    data_overview.push({
      name: test_name,
      data: [{
        x: 0,
        y: final_results['average'],
        min: final_results['min'],
        max: final_results['max'],
        robot: test_list[test_name]['robot'],
        planer: test_list[test_name]['planer_id'],
        planning_method: test_list[test_name]['planning_method'],
        jump_threshold: test_list[test_name]['jump_threshold'],
        eef_step: test_list[test_name]['eef_step']
      }]
    });
    data_categories['speed'].push({
      name: test_name,
      data: [{
        x: 0,
        y: category_results['speed']['average'],
        min: category_results['speed']['min'],
        max: category_results['speed']['max'],
        robot: test_list[test_name]['robot'],
        planer: test_list[test_name]['planer_id'],
        planning_method: test_list[test_name]['planning_method'],
        jump_threshold: test_list[test_name]['jump_threshold'],
        eef_step: test_list[test_name]['eef_step']
      }]
    });
    data_categories['efficiency'].push({
      name: test_name,
      data: [{
        x: 0,
        y: category_results['efficiency']['average'],
        min: category_results['efficiency']['min'],
        max: category_results['efficiency']['max'],
        robot: test_list[test_name]['robot'],
        planer: test_list[test_name]['planer_id'],
        planning_method: test_list[test_name]['planning_method'],
        jump_threshold: test_list[test_name]['jump_threshold'],
        eef_step: test_list[test_name]['eef_step']
      }]
    });
    data_categories['resources'].push({
      name: test_name,
      data: [{
        x: 0,
        y: category_results['resources']['average'],
        min: category_results['resources']['min'],
        max: category_results['resources']['max'],
        robot: test_list[test_name]['robot'],
        planer: test_list[test_name]['planer_id'],
        planning_method: test_list[test_name]['planning_method'],
        jump_threshold: test_list[test_name]['jump_threshold'],
        eef_step: test_list[test_name]['eef_step']
      }]
    });
  });

  chart_compare_overview = new Highcharts.Chart({
    chart: {
      renderTo: 'overview',
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
    series: data_overview,
    plotOptions: {
      column: {
        pointPadding: 0,
        groupPadding: 0,
        borderWidth: 0
      }
    }
  });
  chart_compare_category_speed = new Highcharts.Chart({
    chart: {
      renderTo: 'speed',
      defaultSeriesType: 'column',
      type: 'column',
      zoomType: 'xy'
    },
    title: {
      text: ''
    },
    yAxis: {
      title: {
        text: 'Percentage [%]'
      }
    },
    xAxis: {
      labels: {
        enabled: false
      },
      minTickInterval: 0
    },
    tooltip: plot_tooltip,
    series: data_categories['speed'],
    plotOptions: {
      column: {
        pointPadding: 0,
        groupPadding: 0,
        borderWidth: 0
      }
    }
  });
  chart_compare_category_efficiency = new Highcharts.Chart({
    chart: {
      renderTo: 'efficiency',
      defaultSeriesType: 'column',
      type: 'column',
      zoomType: 'xy'
    },
    title: {
      text: ''
    },
    yAxis: {
      title: {
        text: 'Percentage [%]'
      }
    },
    xAxis: {
      labels: {
        enabled: false
      },
      minTickInterval: 0
    },
    tooltip: plot_tooltip,
    series: data_categories['efficiency'],
    plotOptions: {
      column: {
        pointPadding: 0,
        groupPadding: 0,
        borderWidth: 0
      }
    }
  });
  chart_compare_category_resources = new Highcharts.Chart({
    chart: {
      renderTo: 'resources',
      defaultSeriesType: 'column',
      type: 'column',
      zoomType: 'xy'
    },
    title: {
      text: ''
    },
    yAxis: {
      title: {
        text: 'Percentage [%]'
      }
    },
    xAxis: {
      labels: {
        enabled: false
      },
      minTickInterval: 0
    },
    tooltip: plot_tooltip,
    series: data_categories['resources'],
    plotOptions: {
      column: {
        pointPadding: 0,
        groupPadding: 0,
        borderWidth: 0
      }
    }
  });
  showBestTest();
}

function changeWeight(category, weight) {
  var final_results = {
    average: []
  };
  var results_temp = jQuery.extend(true, {}, results);

  for (var i = 0; i < results_temp[category]['average'].length; i++) {
    results_temp[category]['average'][i] *= weight;
    results_temp[category]['min'][i] *= weight;
    results_temp[category]['max'][i] *= weight;
    chart_compare_category_speed.series[i].setData([(results_temp['speed']['average'][i]).round(1)]);
    chart_compare_category_efficiency.series[i].setData([(results_temp['efficiency']['average'][i]).round(1)]);
    chart_compare_category_resources.series[i].setData([(results_temp['resources']['average'][i]).round(1)]);

    var category_count = 0;
    $.each(results_temp, function (name, value) {
      if (value['average'] != 0) {
        category_count++;
      }
    });

    final_results['average'].push(((results_temp['speed']['average'][i] + results_temp['efficiency']['average'][i] + results_temp['resources']['average'][i]) / category_count).round(1));
    chart_compare_overview.series[i].setData([final_results['average'][i]]);
  }
  showBestTest();
}

function showBestTest() {
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

  $.each(chart_compare_category_speed.series, function (index, data) {
    if (data['data'][0].y > results['speed']['value']) {
      results['speed']['value'] = data['data'][0].y;
      results['speed']['name'] = data.name;
    }
  });
  $.each(chart_compare_category_efficiency.series, function (index, data) {
    if (data['data'][0].y > results['efficiency']['value']) {
      results['efficiency']['value'] = data['data'][0].y;
      results['efficiency']['name'] = data.name;
    }
  });
  $.each(chart_compare_category_resources.series, function (index, data) {
    if (data['data'][0].y > results['resources']['value']) {
      results['resources']['value'] = data['data'][0].y;
      results['resources']['name'] = data.name;
    }
  });
  $.each(chart_compare_overview.series, function (index, data) {
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
}