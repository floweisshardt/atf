var TestList = {
  name: 'test_list',
  convert: function (data) {
    var new_test_list = {};
    $.each(data, function (index, values) {
      $.each(values, function (index, values) {
        new_test_list[index] = values;
      });
    });
    return new_test_list;
  },
  show: function () {
    this.init();
    $('#test_list_content').show();
  },
  init: function () {
    var test_list = FileStorage.readData(this.name);
    var test_list_div = $('#test_list_content').find('#test_list');
    var compare_test_option = $('#compare_test_option');
    var test_list_compare_selection_test = compare_test_option.find('#select_test_config');
    var test_list_compare_selection_scene = compare_test_option.find('#select_scene_config');
    var this_class = this;

    test_list_div.empty();
    test_list_compare_selection_test.empty().append('<option>None</option>').selectpicker('refresh');

    test_list_compare_selection_scene.empty().append('<option>None</option>').selectpicker('refresh');

    var test_config_names = [];
    var scene_config_names = [];
    var number = 1;

    $.each(test_list, function (test_name, test_data) {
      var test_name_full = test_name.split('_');
      var upload_status;
      var table_row_error;
      var test_error;
      var button_disabled;
      var checkbox_disabled;
      var test = FileStorage.readData(test_name);

      if (!test) {
        upload_status = '<span class="glyphicon glyphicon-alert" aria-hidden="true"></span><span class="sr-only">Error: </span> File not found!';
        table_row_error = '<tr class="danger">';
        test_error = '';
        button_disabled = ' disabled="disabled"';
        checkbox_disabled = ' disabled="disabled"';
      } else {
        upload_status = '<span class="glyphicon glyphicon-ok" aria-hidden="true"></span><span class="sr-only">No error:</span> No errors!';
        button_disabled = '';
        var error = this_class.checkForErrors(test);
        if (error[0] === 'error') {
          test_error = '<span class="glyphicon glyphicon-exclamation-sign" aria-hidden="true"></span><span class="sr-only">Error: </span> ' + error[1];
          table_row_error = '<tr class="warning">';
          checkbox_disabled = ' disabled="disabled"';
        } else if (error[0] === 'planning') {
          test_error = '<span class="glyphicon glyphicon-exclamation-sign" aria-hidden="true"></span><span class="sr-only">Error: </span> ' + error[1];
          table_row_error = '<tr class="warning">';
          checkbox_disabled = ' disabled="disabled"';
        } else {
          test_error = upload_status;
          table_row_error = '<tr>';
          checkbox_disabled = button_disabled;

          if ($.inArray(test_data['test_config'], test_config_names) === -1) {
            test_config_names.push(test_data['test_config']);
            test_list_compare_selection_test.prop('disabled', false).append('<option>' + test_data['test_config'] + '</option>').selectpicker('refresh');
          }

          if ($.inArray(test_data['scene_config'], scene_config_names) === -1) {
            scene_config_names.push(test_data['scene_config']);
            test_list_compare_selection_scene.append('<option>' + test_data['scene_config'] + '</option>').selectpicker('refresh');
          }
        }
      }

      test_list_div.append(table_row_error +
        '<td><div class="checkbox-inline"><label><input type="checkbox" value="' + test_name + '"' + checkbox_disabled + '></label></div></td>' +
        '<td>' + number + '</td>' +
        '<td>' + test_name + '</td>' +
        '<td>Testsuite ' + test_name_full[0].replace(/^\D+/g, '') + '</td>' +
        '<td>Test ' + test_name_full[1].replace(/^\D+/g, '') + '</td>' +
        '<td class="test_config">' + test_data['test_config'] + '</td>' +
        '<td class="scene_config">' + test_data['scene_config'] + '</td>' +
        '<td class="robot_name">' + test_data['robot'] + '</td>' +
        '<td>' + upload_status + '</td><td>' + test_error + '</td>' +
        '<td><button id="button_detail" type="button" class="btn btn-primary" data-target="#detail_test" data-toggle="modal" data-name="' + test_name + '"' + button_disabled + '>Details</button></td>');
      number++;
    });
  },
  summarize: function () {
    var test_list = FileStorage.readData(this.name);
    var this_class = this;

    $.each(test_list, function (test_name, test_config) {
      var test_data_complete = {};
      var errors = {
        planning: {},
        error: 0
      };
      $.each(test_config['subtests'], function (index, subtest_name) {
        var test_data = FileStorage.readData(subtest_name);
        if (!test_data) return true;

        $.each(test_data, function (level_1, level_1_data) {
          if (!(level_1 in test_data_complete) && level_1 != 'error') {
            test_data_complete[level_1] = {};
          }

          var error = this_class.checkForErrors(test_data);
          if (error[0] === 'planning') {
            if (!errors['planning'].hasOwnProperty(level_1)) {
              errors['planning'][level_1] = 0;
            }
            errors['planning'][level_1] += 1;
            return true;
          } else if (error[0] === 'error') {
            errors['error'] += 1;
            return true;
          }

          /*
           Level 1: Testblock name
           Level 2: Metric name
           Level 3: - resources: node name
                    - obstacle_distance / path_length: link name
                    - time: values (min, max, average)
           Level 4: - resources: resource name
                    - obstacle_distance / path_length: values (min, max, average)
           Level 5: - resources: values (min, max, average)
           Level 6: - resources io / network: category values
           */

          $.each(level_1_data, function (level_2, level_2_data) {
            if (!(level_2 in test_data_complete[level_1])) {
              test_data_complete[level_1][level_2] = {};
            }

            if (level_2_data instanceof Object) {
              $.each(level_2_data, function (level_3, level_3_data) {
                if (!(level_3 in test_data_complete[level_1][level_2])) {
                  test_data_complete[level_1][level_2][level_3] = {};
                }

                if (level_3_data instanceof Object) {
                  // Resources
                  $.each(level_3_data, function (level_4, level_4_data) {
                    if (!(level_4 in test_data_complete[level_1][level_2][level_3])) {
                      test_data_complete[level_1][level_2][level_3][level_4] = {};
                      test_data_complete[level_1][level_2][level_3][level_4]['max'] = [];
                      test_data_complete[level_1][level_2][level_3][level_4]['min'] = [];
                      test_data_complete[level_1][level_2][level_3][level_4]['average'] = [];
                    }

                    if (level_4_data['average'] instanceof Array) {
                      // IO & Network
                      if (test_data_complete[level_1][level_2][level_3][level_4]['max'].length === 0) {
                        for (var x = 0; x < level_4_data['max'].length; x++) {
                          test_data_complete[level_1][level_2][level_3][level_4]['max'].push([]);
                          test_data_complete[level_1][level_2][level_3][level_4]['min'].push([]);
                          test_data_complete[level_1][level_2][level_3][level_4]['average'].push([]);
                        }
                      }

                      for (x = 0; x < level_4_data['max'].length; x++) {
                        test_data_complete[level_1][level_2][level_3][level_4]['max'][x].push(level_4_data['max'][x]);
                        test_data_complete[level_1][level_2][level_3][level_4]['min'][x].push(level_4_data['min'][x]);
                        test_data_complete[level_1][level_2][level_3][level_4]['average'][x].push(level_4_data['average'][x]);
                      }
                      // CPU & Mem
                    } else {
                      test_data_complete[level_1][level_2][level_3][level_4]['max'].push(level_4_data['max']);
                      test_data_complete[level_1][level_2][level_3][level_4]['min'].push(level_4_data['min']);
                      test_data_complete[level_1][level_2][level_3][level_4]['average'].push(level_4_data['average']);
                    }
                  });
                  // Path length & Obstacle distance
                } else {
                  if ($.isEmptyObject(test_data_complete[level_1][level_2][level_3])) {
                    test_data_complete[level_1][level_2][level_3]['max'] = [];
                    test_data_complete[level_1][level_2][level_3]['min'] = [];
                    test_data_complete[level_1][level_2][level_3]['average'] = [];
                  }
                  test_data_complete[level_1][level_2][level_3]['max'].push(level_3_data);
                  test_data_complete[level_1][level_2][level_3]['min'].push(level_3_data);
                  test_data_complete[level_1][level_2][level_3]['average'].push(level_3_data);

                }
              });
            } else {
              // Time
              if ($.isEmptyObject(test_data_complete[level_1][level_2])) {
                test_data_complete[level_1][level_2]['max'] = [];
                test_data_complete[level_1][level_2]['min'] = [];
                test_data_complete[level_1][level_2]['average'] = [];
              }
              test_data_complete[level_1][level_2]['max'].push(level_2_data);
              test_data_complete[level_1][level_2]['min'].push(level_2_data);
              test_data_complete[level_1][level_2]['average'].push(level_2_data);
            }
          });
        });
        FileStorage.removeData(subtest_name);
      });

      $.each(test_data_complete, function (level_1, level_1_data) {
        $.each(level_1_data, function (level_2, level_2_data) {
          $.each(level_2_data, function (level_3, level_3_data) {
            if (!(Array.isArray(level_3_data))) {
              $.each(level_3_data, function (level_4, level_4_data) {
                if (!(Array.isArray(level_4_data))) {
                  // Resources
                  $.each(level_4_data, function (level_5, level_5_data) {
                    if (typeof level_5_data[0] === 'undefined') {
                      delete test_data_complete[level_1][level_2][level_3][level_4];
                      return true;
                    }
                    if (typeof level_5_data[0][0] === 'undefined') {
                      // CPU & Mem
                      var value = 0;
                      if (level_5 === 'max') value = math.max(level_5_data);
                      else if (level_5 === 'min') value = math.min(level_5_data);
                      else if (level_5 === 'average') value = math.mean(level_5_data);
                      test_data_complete[level_1][level_2][level_3][level_4][level_5] = value
                    } else {
                      // IO & Network
                      $.each(level_5_data, function (level_6, level_6_data) {
                        var value = 0;
                        if (level_5 === 'max') value = math.max(level_6_data);
                        else if (level_5 === 'min') value = math.min(level_6_data);
                        else if (level_5 === 'average') value = math.mean(level_6_data);
                        test_data_complete[level_1][level_2][level_3][level_4][level_5][level_6] = value
                      });
                    }
                  });
                } else {
                  // Path length & obstacle distance
                  var value = 0;
                  if (level_4 === 'max') value = math.max(level_4_data);
                  else if (level_4 === 'min') value = math.min(level_4_data);
                  else if (level_4 === 'average') value = math.mean(level_4_data);
                  test_data_complete[level_1][level_2][level_3][level_4] = value
                }
              });
            } else {
              // Time
              var value = 0;
              if (level_3 === 'max') value = math.max(level_3_data);
              else if (level_3 === 'min') value = math.min(level_3_data);
              else if (level_3 === 'average') value = math.mean(level_3_data);
              test_data_complete[level_1][level_2][level_3] = value
            }
          });
        });
      });

      // Check for errors
      $.each(errors['planning'], function (testblock_name, errors) {
        if (errors === test_config['subtests'].length) {
          test_data_complete[testblock_name]['status'] = 'error';
          return false;
        } else if ((errors['error'] + errors['planning']) === test_config['subtests'].length) {
          test_data_complete[testblock_name]['status'] = 'error';
        }
      });

      if (errors['error'] === test_config['subtests'].length) {
        test_data_complete = {};
        test_data_complete['error'] = 'An error occured outside monitored testblocks. Aborted analysis...';
      }

      FileStorage.removeData(test_name);
      if (Object.keys(test_data_complete).length != 0) {
        FileStorage.writeData(test_name, test_data_complete);
      }
    });
  },
  checkForErrors: function (file) {
    var error = '';
    if (file.hasOwnProperty('error')) {
      error = ['error', 'Error(s) during execution!'];
    } else {
      $.each(file, function (testblock_name, testblock_value) {
        if (testblock_value.hasOwnProperty('status') && testblock_value['status'] === 'error') {
          error = ['planning', 'Planning error in testblock "' + testblock_name + '"!'];
          return false;
        }
      });
    }
    return error;
  },
  showDetails: function (name) {
    var test_detail_div = $('#detail_test');
    var test_name_split = name.split('_');
    test_detail_div.find('.modal-title').html('Details Testsuite ' + test_name_split[0].replace(/^\D+/g, '') + ' - Test ' + test_name_split[1].replace(/^\D+/g, ''));
    var test_details = test_detail_div.find('#detail_panel');
    var test_details_tab_content = test_details.find('.tab-content');
    test_details.hide();

    // Get test data
    var test_results = FileStorage.readData(name);

    // Get test list
    var test_list = FileStorage.readData('test_list');
    var test_data = test_list[name];

    var configuration_div = test_detail_div.find('#detail_configuration');
    configuration_div.empty();

    var status_div = test_detail_div.find('#detail_status');
    status_div.empty();

    var first_entry = true;
    var error = false;

    var plot_tooltip = {
      formatter: function () {
        if (this.series.name.indexOf('variation') != -1) return false;
        var o = this.point.options;

        return '<b>' + this.series.name + '</b><br>' +
          'Average: ' + this.y + '<br>' +
          'Minimum: ' + o.min + '<br>' +
          'Maximum: ' + o.max + '<br>';
      }
    };

    var categories = {
      io: ['Read count',
        'Write count',
        'Kilobytes read',
        'Kilobytes wrote'],
      network: ['Kilobytes sent',
        'Kilobytes received',
        'Packets sent',
        'Packets received',
        'Errors received',
        'Errors sent',
        'Packets dropped: Received',
        'Packets dropped: Sent']
    };

    var plot_options = {
      cpu: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'CPU'
        },
        yAxis: {
          title: {
            text: 'Average consumption [%]'
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
      mem: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Memory'
        },
        yAxis: {
          title: {
            text: 'Average consumption [%]'
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
      io: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Disk IO operations'
        },
        yAxis: {},
        xAxis: {
          labels: {}
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
      network: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Network traffic'
        },
        yAxis: {},
        xAxis: {
          labels: {}
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
      time: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Time'
        },
        yAxis: {
          title: {
            text: 'Time [s]'
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
      path_length: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Path length'
        },
        yAxis: {
          title: {
            text: 'Path length [m]'
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
      obstacle_distance: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Minimal distance to obstacles'
        },
        yAxis: {
          title: {
            text: 'Distance [m]'
          }
        },
        xAxis: {
          labels: {
            enabled: false
          }
        },
        plotOptions: {},
        tooltip: plot_tooltip
      }
    };

    $.each(test_data, function (name) {
      if (name === 'subtests') {
        return true;
      }
      configuration_div.append('<li><b>' + name.capitalize().replace('_', ' ') + ':</b> ' + test_data[name] + '</li>');
    });

    var data_per_test = {};

    $.each(test_results, function (testblock_name, testblock_data) {

      if (testblock_data.hasOwnProperty('status') && testblock_data['status'] === 'error') {
        status_div.append('<div class="alert alert-danger" role="alert">Planning error in testblock "' + testblock_name + '"!</div>');
        error = true;
        return false;
      } else if (testblock_name === 'error') {
        status_div.append('<div class="alert alert-danger" role="alert">An error occured outside monitored testblocks. Evaluation could not be finished!</div>');
        error = true;
        return false;
      }

      var data_per_testblock = {};

      $.each(testblock_data, function (level_2, level_2_data) {
        if (level_2_data.hasOwnProperty('max')) {
          // Time
          if (!data_per_test.hasOwnProperty(level_2)) data_per_test[level_2] = [];
          data_per_test[level_2].push({
            name: testblock_name,
            data: [{
              x: 0,
              y: level_2_data['average'].round(3),
              min: level_2_data['min'].round(3),
              max: level_2_data['max'].round(3)
            }]
          }, {
            name: testblock_name + '_variation',
            type: 'errorbar',
            data: [{
              low: level_2_data['min'].round(3),
              high: level_2_data['max'].round(3)
            }]
          });
        } else {
          $.each(level_2_data, function (level_3, level_3_data) {
            if (level_3_data.hasOwnProperty('max')) {
              // Path length & obstacle distance
              if (!data_per_testblock.hasOwnProperty(level_2)) data_per_testblock[level_2] = [];
              data_per_testblock[level_2].push({
                name: level_3,
                data: [{
                  x: 0,
                  y: level_3_data['average'].round(3),
                  min: level_3_data['min'].round(3),
                  max: level_3_data['max'].round(3)
                }]
              }, {
                name: level_3 + '_variation',
                type: 'errorbar',
                data: [{
                  low: level_3_data['min'].round(3),
                  high: level_3_data['max'].round(3)
                }]
              });
            } else {
              $.each(level_3_data, function (level_4, level_4_data) {
                // Resources
                if (typeof level_4_data['max'][0] === 'undefined') {
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
          });
        }
      });

      if (Object.keys(data_per_testblock).length != 0) {
        var active_class;
        if (first_entry) {
          test_details.find('.nav-tabs').empty();
          test_details.find('.tab-content').empty();
          active_class = 'active';
          first_entry = false;
        } else
          active_class = '';

        test_details.find('.nav-tabs').append('<li role="presentation" class="' + active_class + '"><a href="#details_' + testblock_name + '" aria-controls="details_' + testblock_name + '" role="tab" data-toggle="tab">' + testblock_name + '</a></li>');
        test_details.find('.tab-content').append('<div role="tabpanel" class="tab-pane ' + active_class + '" id="details_' + testblock_name + '"></div>');
      }

      $.each(data_per_testblock, function (metric_name, data) {
        if (data.length != 0) {
          test_details.show();

          var testblock_tab_content = test_details_tab_content.find('#details_' + testblock_name);
          testblock_tab_content.append('<div class="panel panel-info"><div class="panel-heading"></div>' +
            '<div class="panel-body"><div id="details_' + testblock_name + '_' + metric_name + '_content" class="plot"></div></div></div>');
          if (categories.hasOwnProperty(metric_name)) plot_options[metric_name]['xAxis']['categories'] = categories[metric_name];
          else plot_options[metric_name]['xAxis']['categories'] = [];

          $('#details_' + testblock_name).find('#details_' + testblock_name + '_' + metric_name + '_content').highcharts({
            chart: plot_options[metric_name]['chart'],
            title: plot_options[metric_name]['title'],
            xAxis: plot_options[metric_name]['xAxis'],
            yAxis: plot_options[metric_name]['yAxis'],
            tooltip: plot_options[metric_name]['tooltip'],
            series: data,
            plotOptions: plot_options[metric_name]['plotOptions']
          });
        }
      });
    });

    var details_per_test_panel = $('#details_per_test');
    details_per_test_panel.empty();

    $.each(data_per_test, function (metric_name, data) {
      if (data.length != 0) test_details.show();
      details_per_test_panel.append('<div class="panel panel-primary"><div class="panel-heading"></div>' +
      '<div class="panel-body"><div id="details_' + metric_name + '_content" class="plot"></div></div></div>');
      $('#details_' + metric_name + '_content').highcharts({
        chart: plot_options[metric_name]['chart'],
        title: plot_options[metric_name]['title'],
        xAxis: plot_options[metric_name]['xAxis'],
        yAxis: plot_options[metric_name]['yAxis'],
        tooltip: plot_options[metric_name]['tooltip'],
        series: data,
        plotOptions: plot_options[metric_name]['plotOptions']
      });
    });

    if (!error) {
      status_div.append('<div class="alert alert-success" role="alert">No error during evaluation!</div>');
    }
  }
};