function getJSON(folder, filename, callback) {
    $.getJSON(folder + filename + ".json")
        .done(function(json) {
            console.log("Request suceeded");
            writeDataToStorage(filename, json);
            if(callback) {
                callback();
            }
        })
        .fail(function(jqxhr, textStatus, error) {
            var err = textStatus + ", " + error;
            console.log("Request failed: " + err);
        });
}

function drawTestList() {
    var test_list = getDataFromStorage("test_list");
    var test_list_div = $('#test_list_content').find('#test_list');
    test_list_div.empty();

    var number = 1;
    var upload_status;

    test_list.forEach(function(test_data) {
        var test_name = Object.keys(test_data)[0];
        var test_name_full = test_name.split("_");
        if (!getDataFromStorage(test_name)) {
            upload_status = '<span class="glyphicon glyphicon-alert" aria-hidden="true"></span><span class="sr-only">Error: </span> File not found!';
            test_list_div.append('<tr class="danger"><td>' + (number) + '</td><td>' + test_name + '</td><td>Testsuite ' + test_name_full[0].replace(/^\D+/g, '') + '</td><td>Test ' + test_name_full[1].replace(/^\D+/g, "") + '</td><td>' + test_data[test_name]["test_config"] + '</td><td>' + upload_status + '</td><td><button type="button" class="btn btn-default" data-target="#test_detail" data-toggle="modal" data-name="' + test_name + '" disabled>Details</button></td>');
        } else {
            upload_status = '<span class="glyphicon glyphicon-ok" aria-hidden="true"></span><span class="sr-only">No error:</span> No errors!';
            test_list_div.append('<tr><td>' + (number) + '</td><td>' + test_name + '</td><td>Testsuite ' + test_name_full[0].replace(/^\D+/g, '') + '</td><td>Test ' + test_name_full[1].replace(/^\D+/g, "") + '</td><td>' + test_data[test_name]["test_config"] + '</td><td>' + upload_status + '</td><td><button type="button" class="btn btn-default" data-target="#test_detail" data-toggle="modal" data-name="' + test_name + '">Details</button></td>');
        }
        number++;
    });
}

function drawTestDetails(test_name) {
    var test_detail = $('#test_detail');
    var test_name_split = test_name.split("_");
    test_detail.find('.modal-title').html("Details Testsuite " + test_name_split[0].replace(/^\D+/g, "") + " - Test " + test_name_split[1].replace(/^\D+/g, ""));

    // Get test data
    var test_results = getDataFromStorage(test_name);

    // Get test list
    var test_list = getDataFromStorage("test_list");
    var test_data = JSON;
    test_list.forEach(function(data) {
       if (Object.keys(data)[0] === test_name) {
           test_data = data;
           return false;
       }
    });

    var resources_div = test_detail.find('#resources');
    var path_length_div = test_detail.find('#path_length');
    var time_div = test_detail.find('#time');
    var status_div = test_detail.find('#status');

    resources_div.hide();
    time_div.hide();
    path_length_div.hide();

    var first_entry = true;
    var error = false;

    var plot_tooltip = {
        'formatter': function () {
            var o = this.point.options;

            return '<b>' + this.series.name + '</b><br>' +
                'Average: ' + this.y + '<br>' +
                'Minimum: ' + o.min + '<br>' +
                'Maximum: ' + o.max + '<br>';
        }
    };

    var path_lengths = [];
    var path_lengths_drilldowns = [];
    var times = [];

    var configuration_div = test_detail.find('#configuration');
    configuration_div.empty();
    configuration_div.append('<li>Scene config: ' + test_data[test_name]["scene_config"] + '</li>');
    configuration_div.append('<li>Test config: ' + test_data[test_name]["test_config"] + '</li>');
    configuration_div.append('<li>Robot: ' + test_data[test_name]["robot"] + '</li>');
    configuration_div.append('<li>Planer ID: ' + test_data[test_name]["planer_id"] + '</li>');
    configuration_div.append('<li>Planning Method: ' + test_data[test_name]["planning_method"] + '</li>');
    configuration_div.append('<li>Jump threshold: ' + test_data[test_name]["jump_threshold"] + '</li>');
    configuration_div.append('<li>EEF_step: ' + test_data[test_name]["eef_step"] + '</li>');

    $.each(test_results, function(index, value) {
        var testblock_name = index;
        var testblock_metrics = value;

        if (testblock_metrics.hasOwnProperty("status") && testblock_metrics["status"] === "error") {
            status_div.empty();
            status_div.append('<div class="alert alert-danger" role="alert">An error occured in testblock "' + testblock_name + '"!</div>');
            error = true;
        } else if (testblock_name === "Error") {
            status_div.empty();
            status_div.append('<div class="alert alert-danger" role="alert">An error occured outside monitored testblocks. Evaluation could not be executed!</div>');
            error = true;
        }

        if (testblock_metrics.hasOwnProperty("resources")) {
            if (first_entry) {
                resources_div.find('.nav-tabs').empty();
                resources_div.find('.tab-content').empty();
                resources_div.find('.nav-tabs').append('<li role="presentation" class="active"><a href="#' + testblock_name + '" aria-controls="' + testblock_name + '" role="tab" data-toggle="tab">' + testblock_name + '</a></li>');
                resources_div.find('.tab-content').append('<div role="tabpanel" class="tab-pane active" id="' + testblock_name + '"></div>');
                resources_div.show();
                first_entry = false;
            } else {
                resources_div.find('.nav-tabs').append('<li role="presentation"><a href="#' + testblock_name + '" aria-controls="' + testblock_name + '" role="tab" data-toggle="tab">' + testblock_name + '</a></li>');
                resources_div.find('.tab-content').append('<div role="tabpanel" class="tab-pane" id="' + testblock_name + '"></div>');
            }

            var cpu_nodes = [];
            var mem_nodes = [];
            var io_nodes = [];
            var net_nodes = [];
            $.each(testblock_metrics["resources"], function(index, value) {
                var node_name = index;
                var node_resources = value;

                if (node_resources.hasOwnProperty("cpu")) {
                    cpu_nodes.push({
                        'name': node_name,
                        'data': [{
                            'x': 0,
                            'y': node_resources["cpu"]["average"],
                            'min': node_resources["cpu"]["min"],
                            'max': node_resources["cpu"]["max"]
                        }]
                    });
                }
                if (node_resources.hasOwnProperty("mem")) {
                    mem_nodes.push({
                        'name': node_name,
                        'data': [{
                            'x': 0,
                            'y': node_resources["mem"]["average"],
                            'min': node_resources["mem"]["min"],
                            'max': node_resources["mem"]["max"]
                        }]
                    });
                }
                if (node_resources.hasOwnProperty("io")) {
                    io_nodes.push({
                        'name': node_name,
                        'data': [{
                            'x': 0,
                            'y': node_resources["io"]["average"][0],
                            'min': node_resources["io"]["min"][0],
                            'max': node_resources["io"]["max"][0]
                        }, {
                            'x': 1,
                            'y': node_resources["io"]["average"][1],
                            'min': node_resources["io"]["min"][1],
                            'max': node_resources["io"]["max"][1]
                        }, {
                            'x': 2,
                            'y': node_resources["io"]["average"][2],
                            'min': node_resources["io"]["min"][2],
                            'max': node_resources["io"]["max"][2]
                        }, {
                            'x': 3,
                            'y': node_resources["io"]["average"][3],
                            'min': node_resources["io"]["min"][3],
                            'max': node_resources["io"]["max"][3]
                        }]
                    });
                }
                if (node_resources.hasOwnProperty("network")) {
                    net_nodes.push({
                        'name': node_name,
                        'data': [{
                            'x': 0,
                            'y': node_resources["network"]["average"][0],
                            'min': node_resources["network"]["min"][0],
                            'max': node_resources["network"]["max"][0]
                        }, {
                            'x': 1,
                            'y': node_resources["network"]["average"][1],
                            'min': node_resources["network"]["min"][1],
                            'max': node_resources["network"]["max"][1]
                        }, {
                            'x': 2,
                            'y': node_resources["network"]["average"][2],
                            'min': node_resources["network"]["min"][2],
                            'max': node_resources["network"]["max"][2]
                        }, {
                            'x': 3,
                            'y': node_resources["network"]["average"][3],
                            'min': node_resources["network"]["min"][3],
                            'max': node_resources["network"]["max"][3]
                        }, {
                            'x': 4,
                            'y': node_resources["network"]["average"][4],
                            'min': node_resources["network"]["min"][4],
                            'max': node_resources["network"]["max"][4]
                        }, {
                            'x': 5,
                            'y': node_resources["network"]["average"][5],
                            'min': node_resources["network"]["min"][5],
                            'max': node_resources["network"]["max"][5]
                        }, {
                            'x': 6,
                            'y': node_resources["network"]["average"][6],
                            'min': node_resources["network"]["min"][6],
                            'max': node_resources["network"]["max"][6]
                        }, {
                            'x': 7,
                            'y': node_resources["network"]["average"][7],
                            'min': node_resources["network"]["min"][7],
                            'max': node_resources["network"]["max"][7]
                        }]
                    });
                }
            });

            if (cpu_nodes.length != 0) {
                resources_div.find('.tab-content #' + testblock_name).append('<div class="plot" id="' + testblock_name + '_res_cpu"></div>');
                $('#' + testblock_name + '_res_cpu').highcharts({
                    chart: {
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
                    tooltip: plot_tooltip,
                    series: cpu_nodes
                });
            }

            if (mem_nodes.length != 0) {
                resources_div.find('.tab-content #' + testblock_name).append('<div class="plot" id="' + testblock_name + '_res_mem"></div>');
                $('#' + testblock_name + '_res_mem').highcharts({
                    chart: {
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
                    tooltip: plot_tooltip,
                    series: mem_nodes
                });
            }
            if (io_nodes.length != 0) {
                resources_div.find('.tab-content #' + testblock_name).append('<div class="plot" id="' + testblock_name + '_res_io"></div>');
                $('#' + testblock_name + '_res_io').highcharts({
                    chart: {
                        type: 'column',
                        zoomType: 'xy'
                    },
                    title: {
                        text: 'Disk IO operations'
                    },
                    xAxis: {
                        categories: ['Read count', 'Write count', 'Bytes read', 'Bytes wrote']
                    },
                    tooltip: plot_tooltip,
                    series: io_nodes
                });
            }
            if (net_nodes.length != 0) {
                resources_div.find('.tab-content #' + testblock_name).append('<div class="plot" id="' + testblock_name + '_res_network"></div>');
                $('#' + testblock_name + '_res_network').highcharts({
                    chart: {
                        type: 'column',
                        zoomType: 'xy'
                    },
                    title: {
                        text: 'Network traffic'
                    },
                    xAxis: {
                        categories: ['Bytes sent', 'Bytes received', 'Packets sent', 'Packets received', 'Errors received', 'Errors sent', 'Packets dropped: Received', 'Packets dropped: Sent']
                    },
                    tooltip: plot_tooltip,
                    series: net_nodes
                });
            }
        }

        var path_lengths_drilldowns_data = [];
        var path_length_sum = 0;
        $.each(testblock_metrics, function(index, value) {
            var metric_name = index;
            var metric_values = value;
            if (metric_name.contains("path_length")) {
                path_lengths_drilldowns_data.push([metric_name.split("path_length ")[1], metric_values]);
                path_length_sum += metric_values;
            }
        });
        if (path_lengths_drilldowns_data.length != 0) {
            path_lengths_drilldowns.push({
                'name': testblock_name,
                'id': testblock_name,
                'data': path_lengths_drilldowns_data
            });
            path_lengths.push({
                'name': testblock_name,
                'y': path_length_sum,
                'drilldown': testblock_name
            });
        }

        if (testblock_metrics.hasOwnProperty("time")) {
            times.push({
                'name': testblock_name,
                'y': testblock_metrics["time"]
            });
        }
    });

    if (path_lengths.length != 0) {
        path_length_div.show();
        $('#path_length').highcharts({
            chart: {
                type: 'column'
            },
            title: {
                text: 'Path length'
            },
            subtitle: {
                text: 'Click the columns to view path lengths</a>.'
            },
            xAxis: {
                type: 'category'
            },
            yAxis: {
                title: {
                    text: 'Path length'
                }
            },
            legend: {
                enabled: false
            },
            plotOptions: {
                series: {
                    borderWidth: 0,
                    dataLabels: {
                        enabled: true,
                        format: '{point.y}m'
                    }
                }
            },
            tooltip: {
                headerFormat: '<span style="font-size:11px">{series.name}</span><br>',
                pointFormat: '<span style="color:{point.color}">{point.name}</span>: <b>{point.y}m</b> of total<br/>'
            },
            series: [{
                name: "Path lengths",
                colorByPoint: true,
                data: path_lengths
            }],
            drilldown: {
                series: path_lengths_drilldowns
            }
        });
    }
    if (times.length != 0) {
        time_div.show();
        $('#time').highcharts({
            chart: {
                plotBackgroundColor: null,
                plotBorderWidth: null,
                plotShadow: false,
                type: 'pie'
            },
            title: {
                text: 'Time'
            },
            tooltip: {
                pointFormat: '{series.name}: <b>{point.y}s</b>'
            },
            plotOptions: {
                pie: {
                    allowPointSelect: true,
                    cursor: 'pointer',
                    dataLabels: {
                        enabled: true,
                        format: '<b>{point.name}</b>: {point.y}s',
                        style: {
                            color: 'black'
                        }
                    }
                }
            },
            series: [{
                name: "Time",
                colorByPoint: true,
                data: times
            }]
        })
    }

    if (!error) {
        status_div.empty();
        status_div.append('<div class="alert alert-success" role="alert">No error during evaluation!</div>');
    }
}

$(document).ready( function() {
    $('.btn-file :file').on('fileselect', function(event, numFiles, labels) {
        clearStorage();

        for (var x = 0; x <= (labels.length-1); x++) {
            if (x === (labels.length-1)) {
                getJSON("./data/", labels[x], drawTestList);
            } else {
                getJSON("./data/", labels[x]);
            }
        }
        $('#test_list_content').show();
    });

    if (getDataFromStorage("test_list") != null) {
        drawTestList();
        $('#test_list_content').show();
    }
});
