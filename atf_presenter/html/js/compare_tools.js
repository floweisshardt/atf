var chart_compare_overview;
var chart_compare_category_speed;
var chart_compare_category_efficiency;
var chart_compare_category_resources;

function searchForMaximum(tests) {
    var max = {
        "cpu": 0,
        "mem": 0,
        "io": [0, 0, 0, 0],
        "network": [0, 0, 0, 0, 0, 0, 0, 0],
        "time": 0,
        "path_length": 0,
        "obstacle_distance": 0
    };
    $.each(tests, function (index, test_name) {
        var test_results = getDataFromStorage(test_name);

        $.each(test_results, function(testblock_name, testblock) {
            $.each(testblock, function (resource_name, resource_data) {
                if (resource_data instanceof Object) {
                    $.each(resource_data, function (node_name, node_resources) {
                        if (node_resources instanceof  Object) {
                            // Resourcen
                            $.each(node_resources, function (resource_name, resource_data) {
                                if (resource_data instanceof Array) {
                                    // IO & Network
                                    $.each(resource_data, function (index, value) {
                                        if (value > max[resource_name][index]) {
                                            max[resource_name][index] = value;
                                        }
                                    });
                                    // CPU & Mem
                                } else if (resource_data["max"] > max[resource_name]) {
                                    max[resource_name] = resource_data["max"];
                                }
                            });
                            // Path length & Obstacle distance
                        } else if (node_resources > max[resource_name]) {
                            max[resource_name] = node_resources;
                        }
                    });
                } else {
                    // Time
                    if (resource_data > max[resource_name]) {
                        max[resource_name] = resource_data;
                    }
                }
            });
        });
    });

    return max;
}

function compareTests(tests) {
    var compare_tests = $('#compare_tests');
    var configuration_div = compare_tests.find('#compare_configuration');

    var test_list = getDataFromStorage("test_list");

    var data_overview = [];
    var data_categories = {
        "speed": [],
        "efficiency": [],
        "resources": []
    };

    results = {
        "speed": [],
        "resources": [],
        "efficiency": []
    };

    var weight_speed = 1000;
    var weight_resources = 1000;
    var weight_efficiency = 1000;

    var plot_tooltip = {
        "formatter": function () {
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
                '<b>EEF step: </b>' + o.eef_step + '<br>' +
                '<b>Points: </b>' + this.y);

            return '<b>' + this.series.name + '</b><br>' +
                '<b>Points: </b>' + this.y;
        }
    };

    configuration_div.empty();
    configuration_div.append('<li><b>Scene config: </b>' + test_list[tests[0]]["scene_config"] + '</li>' +
        '<li><b>Test config: </b>' + test_list[tests[0]]["test_config"] + '</li>');

    var max_values = searchForMaximum(tests);
    var data_compare_plot = {};

    $.each(tests, function (index, test_name) {

        var category_results = {
            "speed": 0,
            "efficiency": 0,
            "resources": 0
        };

        var test_results = getDataFromStorage(test_name);
        var temp_testblock = {
            "time": {
                "length": 0,
                "current": 0,
                "total": max_values["time"]
            },
            "path_length": {
                "length": 0,
                "current": 0,
                "total": max_values["path_length"]
            },
            "obstacle_distance": {
                "length": 0,
                "current": 0,
                "total": max_values["obstacle_distance"]
            },
            "resources": {
                "cpu": {
                    "length": 0,
                    "current": 0,
                    "total": max_values["cpu"]
                },
                "mem": {
                    "length": 0,
                    "current": 0,
                    "total": max_values["mem"]
                },
                "io": {
                    "length": 0,
                    "current": 0,
                    "total": max_values["io"]
                },
                "network": {
                    "length": 0,
                    "current": 0,
                    "total": max_values["network"]
                }
            }
        };

        $.each(test_results, function(testblock_name, testblock) {
            if (!(testblock_name in data_compare_plot)) {
                data_compare_plot[testblock_name] = {};
            }

            $.each(testblock, function (resource_name, resource_data) {
                if (resource_data instanceof Object) {
                    $.each(resource_data, function (node_name, node_resources) {
                        if (node_resources instanceof  Object) {
                            // Resourcen
                            $.each(node_resources, function (resource_name, resource_data) {

                                if (!(resource_name in data_compare_plot[testblock_name])) {
                                    data_compare_plot[testblock_name][resource_name] = {};
                                }
                                if (!(node_name in data_compare_plot[testblock_name][resource_name])) {
                                    data_compare_plot[testblock_name][resource_name][node_name] = {};
                                }
                                data_compare_plot[testblock_name][resource_name][node_name][test_name] = resource_data["average"];

                                if (resource_data["average"] instanceof Array) {
                                    // IO & Network
                                    for (var x = 0; x < resource_data.length; x++) {
                                        if (temp_testblock["resources"][resource_name]["total"][x] != 0) {
                                            temp_testblock["resources"][resource_name]["current"] += (temp_testblock["resources"][resource_name]["total"][x] - resource_data["average"][x])/temp_testblock["resources"][resource_name]["total"][x];
                                        }
                                        temp_testblock["resources"][resource_name]["length"]++;
                                    }
                                    // CPU & Mem
                                } else {
                                    if (temp_testblock["resources"][resource_name]["total"] != 0) {
                                        temp_testblock["resources"][resource_name]["current"] += (temp_testblock["resources"][resource_name]["total"] - resource_data["average"])/temp_testblock["resources"][resource_name]["total"];
                                    }
                                    temp_testblock["resources"][resource_name]["length"]++;
                                }
                            });
                            // Path length & Obstacle distance
                        } else {
                            if (!(resource_name in data_compare_plot[testblock_name])) {
                                data_compare_plot[testblock_name][resource_name] = {};
                            }
                            if (!(node_name in data_compare_plot[testblock_name][resource_name])) {
                                data_compare_plot[testblock_name][resource_name][node_name] = {};
                            }
                            data_compare_plot[testblock_name][resource_name][node_name][test_name] = node_resources;
                            if (temp_testblock[resource_name]["total"] != 0) {
                                temp_testblock[resource_name]["current"] += (temp_testblock[resource_name]["total"] - node_resources) / temp_testblock[resource_name]["total"];
                            }
                            temp_testblock[resource_name]["length"]++;
                        }
                    });
                } else {
                    // Time
                    if (!(resource_name in data_compare_plot[testblock_name])) {
                        data_compare_plot[testblock_name][resource_name] = {}
                    }
                    data_compare_plot[testblock_name][resource_name][test_name] = resource_data;
                    if (temp_testblock[resource_name]["total"] != 0) {
                        temp_testblock[resource_name]["current"] += (temp_testblock[resource_name]["total"] - resource_data) / temp_testblock[resource_name]["total"];
                    }
                    temp_testblock[resource_name]["length"]++;
                }
            });
        });

        var temp_categories = {
            "resources": 0,
            "time": 0,
            "path_length": 0,
            "obstacle_distance": 0
        };

        var count_resource_categories = 0;
        var temp_efficiency = 0;

        $.each(temp_testblock, function(name, values) {
            if (values instanceof Object) {
                // Resources
                $.each(values, function(res_name, res_data) {
                    if (res_data["length"] != 0) {
                        temp_categories[name] += res_data["current"]/res_data["length"];
                        count_resource_categories++;
                    }
                });
            } else {
                // Time & Path length & Obstacle distance
                if (values["length"] != 0) {
                    temp_categories[name] = values["current"]/values["length"];
                }
            }
        });

        var temp_resources = temp_categories/count_resource_categories;
        var temp_speed = temp_categories["time"];

        if (temp_categories["path_length"] != 0 && temp_categories["obstacle_distance"] != 0) {
            temp_efficiency = (temp_categories["path_length"] + temp_categories["obstacle_distance"])/2;
        } else {
            temp_efficiency = (temp_categories["path_length"] + temp_categories["obstacle_distance"]);
        }

        results["speed"].push(temp_speed * weight_speed);
        results["efficiency"].push(temp_efficiency * weight_efficiency);
        results["resources"].push(temp_resources * weight_resources);

        category_results["speed"] = round(temp_speed * weight_speed, 3);
        category_results["efficiency"] = round(temp_efficiency * weight_efficiency, 3);
        category_results["resources"] = round(temp_resources * weight_resources, 3);

        var final_results = round((temp_speed * weight_speed + temp_efficiency * weight_efficiency + temp_resources * weight_resources), 3);
        data_overview.push({
            'name': test_name,
            'data': [{
                'x': 0,
                'y': final_results,
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }]
        });
        data_categories["speed"].push({
            'name': test_name,
            'data': [{
                'x': 0,
                'y': category_results["speed"],
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }]
        });
        data_categories["efficiency"].push({
            'name': test_name,
            'data': [{
                'x': 0,
                'y': category_results["efficiency"],
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }]
        });
        data_categories["resources"].push({
            'name': test_name,
            'data': [{
                'x': 0,
                'y': category_results["resources"],
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
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
                text: 'Points'
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
                text: 'Points'
            }
        },
        xAxis: {
            labels: {
                enabled: false
            },
            minTickInterval: 0
        },
        tooltip: plot_tooltip,
        series: data_categories["speed"],
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
                text: 'Points'
            }
        },
        xAxis: {
            labels: {
                enabled: false
            },
            minTickInterval: 0
        },
        tooltip: plot_tooltip,
        series: data_categories["efficiency"],
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
                text: 'Points'
            }
        },
        xAxis: {
            labels: {
                enabled: false
            },
            minTickInterval: 0
        },
        tooltip: plot_tooltip,
        series: data_categories["resources"],
        plotOptions: {
            column: {
                pointPadding: 0,
                groupPadding: 0,
                borderWidth: 0
            }
        }
    });
    //createComparisonGraphs(data_compare_plot);
    showBestTest();
}

function changeWeight(category, weight) {

    var final_results = [];
    var results_temp = jQuery.extend(true, {}, results);

    for (var i = 0; i < results_temp[category].length; i++) {
        results_temp[category][i] *= weight;
        chart_compare_category_speed.series[i].setData([round(results_temp["speed"][i], 3)]);
        chart_compare_category_efficiency.series[i].setData([round(results_temp["efficiency"][i], 3)]);
        chart_compare_category_resources.series[i].setData([round(results_temp["resources"][i], 3)]);

        final_results.push(round(results_temp["speed"][i] + results_temp["efficiency"][i] + results_temp["resources"][i], 3));
        chart_compare_overview.series[i].setData([final_results[i]]);
    }
    showBestTest();
}

function showBestTest() {
    var results = {
        "speed": {
            "name": "",
            "value": 0
        },
        "efficiency": {
            "name": "",
            "value": 0
        },
        "resources": {
            "name": "",
            "value": 0
        },
        "total": {
            "name": "",
            "value": 0
        }
    };

    var compare_tests = $('#compare_tests').find('#test_results');
    var speed = compare_tests.find('#result_overview_speed');
    var efficiency = compare_tests.find('#result_overview_efficiency');
    var resources = compare_tests.find('#result_overview_resources');
    var total = compare_tests.find('#result_overview_total');

    $.each(chart_compare_category_speed.series, function (index, data) {
        if (data["data"][0].y > results["speed"]["value"]) {
            results["speed"]["value"] = data["data"][0].y;
            results["speed"]["name"] = data.name;
        }
    });
    $.each(chart_compare_category_efficiency.series, function (index, data) {
        if (data["data"][0].y > results["efficiency"]["value"]) {
            results["efficiency"]["value"] = data["data"][0].y;
            results["efficiency"]["name"] = data.name;
        }
    });
    $.each(chart_compare_category_resources.series, function (index, data) {
        if (data["data"][0].y > results["resources"]["value"]) {
            results["resources"]["value"] = data["data"][0].y;
            results["resources"]["name"] = data.name;
        }
    });
    $.each(chart_compare_overview.series, function (index, data) {
        if (data["data"][0].y > results["total"]["value"]) {
            results["total"]["value"] = data["data"][0].y;
            results["total"]["name"] = data.name;
        }
    });
    // TODO: Display points in table ???
    speed.empty();
    efficiency.empty();
    resources.empty();
    total.empty();

    speed.append(results["speed"]["name"]);
    efficiency.append(results["efficiency"]["name"]);
    resources.append(results["resources"]["name"]);
    total.append(results["total"]["name"]);
}

function createComparisonGraphs(data) {
    var compare_tests = $('#compare_tests');
    var compare_tests_detail = compare_tests.find('#compare_results_detail');
    var compare_tests_detail_tab_content = compare_tests_detail.find('.tab-content');

    var first_entry = true;
    var compare_categories = {
        "cpu": [],
        "mem": [],
        "io": [],
        "network": [],
        "time": "",
        "path_length": []
    };
    var compare_categories_items = {
        "io": ['Read count',
            'Write count',
            'Kilobytes read',
            'Kilobytes wrote'],
        "network": ['Kilobytes sent',
            'Kilobytes received',
            'Packets sent',
            'Packets received',
            'Errors received',
            'Errors sent',
            'Packets dropped: Received',
            'Packets dropped: Sent']
    };
    var plot_options = {
        "cpu": {
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
                labels: {}
            },
            plotOptions: {}
        },
        "mem": {
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
                labels: {}
            },
            plotOptions: {}
        },
        "io": {
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
            plotOptions: {}
        },
        "network": {
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
            plotOptions: {}
        },
        "time": {
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
            plotOptions: {
                column: {
                    pointPadding: 0,
                    groupPadding: 0,
                    borderWidth: 0
                }
            }
        },
        "path_length": {
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
            xAxis: {},
            plotOptions: {}
        }
    };

    var plot_tooltip = {
        "formatter": function () {

            return '<b>' + this.series.name + '</b>: '+ this.y;
        }
    };

    $.each(data, function (testblock_name, testblock_data) {
        var compare_data = {
            "cpu": [],
            "mem": [],
            "io": [],
            "network": [],
            "time": [],
            "path_length": []
        };

        var active_class;
        if (first_entry) {
            compare_tests_detail.find('.nav-tabs').empty();
            compare_tests_detail.find('.tab-content').empty();
            active_class = "active";
            first_entry = false;
        } else {
            active_class = "";
        }

        compare_tests_detail.find('.nav-tabs').append('<li role="presentation" class="' + active_class + '"><a href="#compare_' + testblock_name + '" aria-controls="compare_' + testblock_name + '" role="tab" data-toggle="tab">' + testblock_name + '</a></li>');
        compare_tests_detail.find('.tab-content').append('<div role="tabpanel" class="tab-pane ' + active_class + '" id="compare_' + testblock_name + '"></div>');

        $.each(testblock_data, function (resource_name, resource_data) {
            var testblock_tab_content = compare_tests_detail_tab_content.find('#compare_' + testblock_name);
            var category_name = resource_name.split('_').join(' ');

            testblock_tab_content.append('<div class="panel panel-primary"><div class="panel-heading">' + category_name + '</div>' +
                '<div class="panel-body"><div id="compare_' + resource_name + '_detail" class="plot"></div></div></div>');

            // TODO: Fix grouped categories for IO & Network
            $.each(resource_data, function (name, data) {
                if (data instanceof Object) {
                    $.each(data, function (test_name, test_data) {
                        var data = [];
                        if (test_data instanceof Array) {
                            // IO & Network
                            if ($.inArray(name, compare_categories[resource_name]) === -1) {
                                compare_categories[resource_name].push({"name": name, "categories": compare_categories_items[resource_name]});
                            }
                            for (var i = 0; i < test_data.length; i++) {
                                data.push(test_data[i]);
                            }
                        } else {
                            /// CPU & Mem & Path length
                            if ($.inArray(name, compare_categories[resource_name]) === -1) {
                                compare_categories[resource_name].push(name);
                            }
                            data.push({
                                'x': compare_categories[resource_name].indexOf(name),
                                'y': test_data
                            });
                        }
                        compare_data[resource_name].push({
                            'name': test_name,
                            'data': data
                        });
                    });
                } else {
                    /// Time
                    compare_data[resource_name].push({
                        'name': name,
                        'data': [{
                            'y': data
                        }]
                    });
                }
            });
        });

        $.each(data[testblock_name], function (resource_name, data) {

            plot_options[resource_name]["xAxis"]["categories"] = compare_categories[resource_name];

            $('#compare_' + testblock_name).find('#compare_' + resource_name + '_detail').highcharts({
                chart: plot_options[resource_name]["chart"],
                title: plot_options[resource_name]["title"],
                xAxis: plot_options[resource_name]["xAxis"],
                yAxis: plot_options[resource_name]["yAxis"],
                tooltip: plot_tooltip,
                series: compare_data[resource_name],
                plotOptions: plot_options[resource_name]["plotOptions"]
            });
        });
    });
}