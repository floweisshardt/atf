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
    //console.log("test_list=", test_list)

    $.each(test_list, function (test_name, test_config) {
      //console.log("test_name=", test_name)
      //console.log("test_config=", test_config)
      var test_data_complete = {};
      var errors = {
        planning: {},
        error: 0
      };
      $.each(test_config['subtests'], function (index, subtest_name) {
        //console.log("subtest_name=", subtest_name)
        var test_data = FileStorage.readData(subtest_name);
        //console.log("test_data=", test_data)
        if (!test_data) return true;

        $.each(test_data, function (testblock_name, testblock_data) {
          //console.log("testblock_name=", testblock_name)
          //console.log("testblock_data=", testblock_data)
          if (!(testblock_name in test_data_complete) && testblock_name != 'error') {
            test_data_complete[testblock_name] = {};
          }

          var error = this_class.checkForErrors(test_data);
          if (error[0] === 'planning') {
            if (!errors['planning'].hasOwnProperty(testblock_name)) {
              errors['planning'][testblock_name] = 0;
            }
            errors['planning'][testblock_name] += 1;
            return true;
          } else if (error[0] === 'error') {
            errors['error'] += 1;
            return true;
          }

          $.each(testblock_data, function (metric_name, metric_data) {
            //console.log("metric_name",metric_name)
            //console.log("metric_data",metric_data)
            if (!(metric_name in test_data_complete[testblock_name])) {
              test_data_complete[testblock_name][metric_name] = {};
            }


            $.each(metric_data, function (metric_key_name, metric_key_data) {
              //console.log("metric_key_name",metric_key_name)
              //console.log("metric_key_data",metric_key_data)
              
              if (metric_key_name == 'data') {
                // if key is data, we'll setup lists for max/min/average
                if (!(metric_key_name in test_data_complete[testblock_name][metric_name])) {
                  test_data_complete[testblock_name][metric_name][metric_key_name] = {};
                }
                
                if ($.isEmptyObject(test_data_complete[testblock_name][metric_name][metric_key_name])) {
                  test_data_complete[testblock_name][metric_name][metric_key_name]['values'] = [];
                }
                test_data_complete[testblock_name][metric_name][metric_key_name]['values'].push(metric_key_data);
              } else {
                // if key is not data with max/min/average, we'll copy the original key_data
                test_data_complete[testblock_name][metric_name][metric_key_name] = metric_key_data
              }
            });
          });
        });
        FileStorage.removeData(subtest_name);
      });

      //console.log("test_data_complete", test_data_complete)
      
      // build max/min/average out of lists
      $.each(test_data_complete, function (testblock_name, testblock_data) {
        $.each(testblock_data, function (metric_name, metric_data) {
          $.each(metric_data, function (metric_key_name, metric_key_data) {
            //console.log("metric_key_name=", metric_key_name)
            //console.log("metric_key_data=", metric_key_data)
            if (metric_key_name == 'data') {
              test_data_complete[testblock_name][metric_name][metric_key_name]['min'] = math.min(metric_key_data['values']);
              test_data_complete[testblock_name][metric_name][metric_key_name]['max'] = math.max(metric_key_data['values']);
              test_data_complete[testblock_name][metric_name][metric_key_name]['average'] = math.mean(metric_key_data['values']);
            }
          });
        });
      });
      
      //console.log("test_data_complete", test_data_complete)
      
      // Check for errors
      var test_failed = 0;
      $.each(errors['planning'], function (testblock_name, errors) {
        test_failed += errors;
        if (errors === test_config['subtests'].length) {
          test_data_complete[testblock_name]['status'] = 'error';
          return false;
        } else if ((errors['error'] + errors['planning']) === test_config['subtests'].length) {
          test_data_complete[testblock_name]['status'] = 'error';
        }
      });
      test_failed += errors['error'];
      if (errors['error'] === test_config['subtests'].length) {
        test_data_complete = {};
        test_data_complete['error'] = 'An error occured outside monitored testblocks. Aborted analysis...';
      }

      FileStorage.removeData(test_name);
      if (Object.keys(test_data_complete).length != 0) {
        test_list[test_name]['tests_failed'] = test_failed;
        FileStorage.writeData(test_name, test_data_complete);
      }
    });
    FileStorage.removeData(this.name);
    FileStorage.writeData(this.name, test_list);
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
      if (name === 'subtests') return true;
      configuration_div.append('<li><b>' + name.capitalize().replace('_', ' ') + ':</b> ' + test_data[name] + '</li>');
    });

    var data_per_test = {};

    $.each(test_results, function (testblock_name, testblock_data) {
      //console.log("testblock_name=", testblock_name);
      //console.log("testblock_data=", testblock_data);

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

      $.each(testblock_data, function (metric, data) {
        //console.log("metric=", metric);
        //console.log("data=", data);
        if (metric == 'time' )
        {
          // Time
          time_data = data['data']
          if (!data_per_test.hasOwnProperty(metric)) data_per_test[metric] = [];
          data_per_test[metric].push({
            name: testblock_name,
            data: [{
              x: 0,
              y: time_data['average'].round(3),
              min: time_data['min'].round(3),
              max: time_data['max'].round(3)
            }]
          }, {
            name: testblock_name + '_variation',
            type: 'errorbar',
            data: [{
              low: time_data['min'].round(3),
              high: time_data['max'].round(3)
            }]
          });
        }
        if (metric == 'path_length')
        {
          // Path Length
          //console.log("data=", data);
          path_length_data = data['data']
          if (!data_per_test.hasOwnProperty(metric)) data_per_test[metric] = [];
          data_per_test[metric].push({
            name: testblock_name + "<br>(" + data['details']['root_frame'] + " to " + data['details']['measured_frame'] + ")",
            data: [{
              x: 0,
              y: path_length_data['average'].round(3),
              min: path_length_data['min'].round(3),
              max: path_length_data['max'].round(3)
            }]
          }, {
            name: testblock_name + '_variation',
            type: 'errorbar',
            data: [{
              low: path_length_data['min'].round(3),
              high: path_length_data['max'].round(3)
            }]
          });
          }
        if (metric == 'resources')
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