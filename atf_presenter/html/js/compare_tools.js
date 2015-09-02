function searchForMaximum(tests) {
    var max = {
        "cpu": 0,
        "mem": 0,
        "io": [0, 0, 0, 0],
        "network": [0, 0, 0, 0, 0, 0, 0, 0]
    };
    $.each(tests, function (index, test_name) {
        var test_results = getDataFromStorage(test_name);

        $.each(test_results, function(testblock_name, testblock) {
            if (testblock.hasOwnProperty("resources")) {
                $.each(testblock["resources"], function (node_name, node_resources) {
                    $.each(node_resources, function (resource_name, resource_data) {
                        if (resource_data["max"] > max[resource_name]) {
                            max[resource_name] = resource_data["max"];
                        } else if (resource_data instanceof Array) {
                            $.each(resource_data, function (index, value) {
                                if (value > max[resource_name][index]) {
                                    max[resource_name][index] = value;
                                }
                            });
                        }
                    });
                });
            }
        });
    });
    return max;
}

function compareTests(tests) {
    var compare_tests = $('#compare_tests');
    var configuration_div = compare_tests.find('#compare_configuration');
    var compare_tests_detail = compare_tests.find('#compare_results_detail');

    var resources_div = compare_tests_detail.find('#compare_resources_detail');
    var resources_panel = compare_tests_detail.find('#panel_compare_detail_resources');

    var path_length_div = compare_tests_detail.find('#compare_path_length_detail');
    var path_length_panel = compare_tests_detail.find('#panel_compare_detail_path_length');

    var time_div = compare_tests_detail.find('#compare_time_detail');
    var time_panel = compare_tests_detail.find('#panel_compare_detail_time');

    resources_panel.hide();
    path_length_panel.hide();
    time_panel.hide();

    var test_list = getDataFromStorage("test_list");

    var data_overview = [];
    var data_categories = [];

    results = {
        "speed": [],
        "resources": [],
        "efficiency": []
    };

    var weight_speed = 10000;
    var weight_resources = 10000;
    var weight_efficiency = 10000;

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

    var first_test = true;
    var active_tab = true;
    var compare_cpu = [];
    var compare_mem = [];
    var compare_io = [];
    var compare_net = [];
    var node_names = [];
    var testblock_names = [];

    var max_values = searchForMaximum(tests);

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
                "total": 500
            },
            "path_length": {
                "length": 0,
                "current": 0,
                "total": 50
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

        $.each(test_results, function (testblock_name, testblock) {

            if (testblock.hasOwnProperty("time")) {
                if (temp_testblock["time"]["total"] != 0) {
                    temp_testblock["time"]["current"] += (temp_testblock["time"]["total"] - testblock["time"]) / temp_testblock["time"]["total"];
                }
                temp_testblock["time"]["length"]++;
            }

            if (testblock.hasOwnProperty("resources")) {

                $.each(testblock["resources"], function (node_name, node_resources) {

                    $.each(node_resources, function (resource_name, resource_data) {

                        if (resource_data["average"] instanceof Array) {
                            for (var x = 0; x < resource_data.length; x++) {
                                if (temp_testblock["resources"][resource_name]["total"][x] != 0) {
                                    temp_testblock["resources"][resource_name]["current"] += (temp_testblock["resources"][resource_name]["total"][x] - resource_data["average"][x])/temp_testblock["resources"][resource_name]["total"][x];
                                }
                                temp_testblock["resources"][resource_name]["length"]++;
                            }
                        } else {
                            if (temp_testblock["resources"][resource_name]["total"] != 0) {
                                temp_testblock["resources"][resource_name]["current"] += (temp_testblock["resources"][resource_name]["total"] - resource_data["average"])/temp_testblock["resources"][resource_name]["total"];
                            }
                            temp_testblock["resources"][resource_name]["length"]++;
                        }
                    });
                });
            }

            $.each(testblock, function (metric_name, metric_values) {
                if (metric_name.contains("path_length")) {
                    if (temp_testblock["path_length"]["total"] != 0) {
                        temp_testblock["path_length"]["current"] += (temp_testblock["path_length"]["total"] - metric_values) / temp_testblock["path_length"]["total"];
                    }
                    temp_testblock["path_length"]["length"]++;
                }
            });
        });

        var temp_resources = 0;

        if (temp_testblock["resources"]["cpu"]["length"] != 0) {
            temp_resources += temp_testblock["resources"]["cpu"]["current"]/temp_testblock["resources"]["cpu"]["length"];
        }
        if (temp_testblock["resources"]["mem"]["length"] != 0) {
            temp_resources += temp_testblock["resources"]["mem"]["current"]/temp_testblock["resources"]["mem"]["length"];
        }
        if (temp_testblock["resources"]["io"]["length"] != 0) {
            temp_resources += temp_testblock["resources"]["io"]["current"]/temp_testblock["resources"]["io"]["length"];
        }
        if (temp_testblock["resources"]["network"]["length"] != 0) {
            temp_resources += temp_testblock["resources"]["network"]["current"]/temp_testblock["resources"]["network"]["length"];
        }

        temp_resources /= 4;
        if (isNaN(temp_resources)) temp_resources = 0;

        var temp_speed = temp_testblock["time"]["current"]/temp_testblock["time"]["length"];
        var temp_efficiency = temp_testblock["path_length"]["current"]/temp_testblock["path_length"]["length"];

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
                'x': index,
                'y': final_results,
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }]
        });
        data_categories.push({
            'name': test_name,
            'data': [{
                'x': 0,
                'y': category_results["speed"],
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }, {
                'x': 1,
                'y': category_results["efficiency"],
                'robot': test_list[test_name]["robot"],
                'planer': test_list[test_name]["planer_id"],
                'planning_method': test_list[test_name]["planning_method"],
                'jump_threshold': test_list[test_name]["jump_threshold"],
                'eef_step': test_list[test_name]["eef_step"]
            }, {
                'x': 2,
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
            }
        },
        yAxis: {
            title: {
                text: 'Points'
            }
        },
        tooltip: plot_tooltip,
        series: data_overview
    });

    chart_compare_categories = new Highcharts.Chart({
        chart: {
            renderTo: 'categories',
            defaultSeriesType: 'column',
            type: 'column',
            zoomType: 'xy'
        },
        title: {
            text: ''
        },
        xAxis: {
            categories: ['Speed', 'Efficiency', 'Resources']
        },
        yAxis: {
            title: {
                text: 'Points'
            }
        },
        tooltip: plot_tooltip,
        series: data_categories
    });

    /*$.each(testblock_names, function (index, name) {
        new Highcharts.Chart({
            chart: {
                renderTo: resources_div.find('.tab-content #' + name),
                defaultSeriesType: 'column',
                type: 'column',
                zoomType: 'xy'
            },
            title: {
                text: ''
            },
            xAxis: {
                categories: ['Speed', 'Efficiency', 'Resources']
            },
            yAxis: {
                title: {
                    text: 'Points'
                }
            },
            tooltip: plot_tooltip,
            series: data_categories
        });
    });*/
}

function changeWeight(category, weight) {

    var final_results = [];

    for (var i = 0; i < results[category].length; i++) {
        results[category][i] *= weight;
        chart_compare_categories.series[i].setData([
            round(results["speed"][i], 3),
            round(results["efficiency"][i], 3),
            round(results["resources"][i], 3)]);

        final_results.push(round(results["speed"][i] + results["efficiency"][i] + results["resources"][i], 3));
        chart_compare_overview.series[i].setData([final_results[i]]);
    }
}