var results = {
    "speed": [],
    "resources": [],
    "efficiency": []
};

var progressbar_value = 0;

function round(number, decimals) {
    return +(Math.round(number + "e+" + decimals) + "e-" + decimals);
}

function getData(folder, files) {
    var list = [];
    for (var x = 0; x < files.length; x++) {
        var filename = files[x];
        var total_files = files.length;
        list.push($.getJSON(folder + filename + ".json")
            .done(onJSONSuccess(filename, total_files))
            .fail(onJSONFail(total_files))
        );
    }
    $.when.all(list).always(function () {
        showTestList();
    });
}

function onJSONFail(file_length) {
    return function (jqxhr, textStatus, error) {
        var err = textStatus + ": " + error;
        console.log("Request failed: " + err);
        updateProgressbar(file_length);
    };
}

function onJSONSuccess(filename, file_length) {
    return function (data) {
        if (filename.contains("test_list")) {
            data = convertTestList(data);
        }
        if (!writeDataToStorage(filename, data)) {
            console.log("Writing to storage failed!");
        } else {
            console.log("Request suceeded");
        }
        updateProgressbar(file_length);
    };
}

function convertTestList(test_list) {
    var new_test_list = {};
    $.each(test_list, function (index, values) {
        $.each(values, function (index, values) {
           new_test_list[index] = values;
        });
    });
    return new_test_list;
}

function updateProgressbar(file_length) {

    var progressbar = $('#file_upload_progressbar');

    if (progressbar.attr('aria-valuenow') >= 100) {
        progressbar_value = 0;
    }

    progressbar_value += (1/file_length)*100;
    var value = round(progressbar_value, 0);

    progressbar.empty();
    progressbar.css('width', value + '%').attr('aria-valuenow', value);
    progressbar.append(value + '%');
}

function showTestList() {
    drawTestList();
    $('#test_list_content').show();
}

function drawTestList() {
    var test_list = getDataFromStorage("test_list");
    var test_list_div = $('#test_list_content').find('#test_list');
    var compare_test_option = $('#compare_test_option');
    var test_list_compare_selection_test = compare_test_option.find('#select_test_config');
    var test_list_compare_selection_scene = compare_test_option.find('#select_scene_config');

    test_list_div.empty();
    test_list_compare_selection_test.empty().append('<option>None</option>').selectpicker('refresh');

    test_list_compare_selection_scene.empty().append('<option>None</option>').selectpicker('refresh');

    var test_config_names = [];
    var scene_config_names = [];

    var number = 1;
    $.each(test_list, function (test_name, test_data) {
        var test_name_full = test_name.split("_");
        var upload_status;
        var table_row_error;
        var test_error;
        var button_disabled;
        var checkbox_disabled;
        var test = getDataFromStorage(test_name);

        if (!test) {
            upload_status = '<span class="glyphicon glyphicon-alert" aria-hidden="true"></span><span class="sr-only">Error: </span> File not found!';
            table_row_error = '<tr class="danger">';
            test_error = '';
            button_disabled = ' disabled="disabled"';
            checkbox_disabled = ' disabled="disabled"';
        } else {
            upload_status = '<span class="glyphicon glyphicon-ok" aria-hidden="true"></span><span class="sr-only">No error:</span> No errors!';
            button_disabled = '';
            var error = checkforError(test);
            if (error[0] === "error") {
                test_error = '<span class="glyphicon glyphicon-exclamation-sign" aria-hidden="true"></span><span class="sr-only">Error: </span> ' + error[1];
                table_row_error = '<tr class="warning">';
                checkbox_disabled = ' disabled="disabled"';
            }else if (error[0] === "planning") {
                test_error = '<span class="glyphicon glyphicon-exclamation-sign" aria-hidden="true"></span><span class="sr-only">Error: </span> ' + error[1];
                table_row_error = '<tr class="warning">';
                checkbox_disabled = ' disabled="disabled"';
            } else {
                test_error = upload_status;
                table_row_error = '<tr>';
                checkbox_disabled = button_disabled;

                if ($.inArray(test_data["test_config"], test_config_names) === -1) {
                    test_config_names.push(test_data["test_config"]);
                    test_list_compare_selection_test.prop("disabled", false).append('<option>' + test_data["test_config"] + '</option>').selectpicker('refresh');
                }

                if ($.inArray(test_data["scene_config"], scene_config_names) === -1) {
                    scene_config_names.push(test_data["scene_config"]);
                    test_list_compare_selection_scene.append('<option>' + test_data["scene_config"] + '</option>').selectpicker('refresh');
                }
            }
        }

        test_list_div.append(table_row_error + '<td><div class="checkbox-inline"><label><input type="checkbox" value="' + test_name + '"' + checkbox_disabled + '></label></div></td><td>' + number + '</td><td>' + test_name + '</td><td>Testsuite ' + test_name_full[0].replace(/^\D+/g, '') + '</td><td>Test ' + test_name_full[1].replace(/^\D+/g, "") + '</td><td class="test_config">' + test_data["test_config"] + '</td><td class="scene_config">' + test_data["scene_config"] + '</td><td class="robot_name">' + test_data["robot"] + '</td><td>' + upload_status + '</td><td>' + test_error + '</td><td><button id="button_detail" type="button" class="btn btn-primary" data-target="#detail_test" data-toggle="modal" data-name="' + test_name + '"' + button_disabled + '>Details</button></td>');
        number++;
    });
}

function checkforError(test_file) {
    var error = "";
    if (test_file.hasOwnProperty("error")) {
        error = ["error", "Error(s) during execution!"];
    } else {
        $.each(test_file, function (testblock_name, testblock_value) {
            if (testblock_value.hasOwnProperty("status") && testblock_value["status"] === "error") {
                error = ["planning", 'Planning error in testblock "' + testblock_name + '"!'];
                return false;
            }
        });
    }
    return error;
}

function drawTestDetails(test_name) {
    var test_detail_div = $('#detail_test');
    var test_name_split = test_name.split("_");
    test_detail_div.find('.modal-title').html("Details Testsuite " + test_name_split[0].replace(/^\D+/g, "") + " - Test " + test_name_split[1].replace(/^\D+/g, ""));
    var test_details = test_detail_div.find('#detail_panel');
    var test_details_tab_content = test_details.find('.tab-content');

    test_details.hide();

    // Get test data
    var test_results = getDataFromStorage(test_name);

    // Get test list
    var test_list = getDataFromStorage("test_list");
    var test_data = test_list[test_name];

    var configuration_div = test_detail_div.find('#detail_configuration');
    configuration_div.empty();

    var status_div = test_detail_div.find('#detail_status');
    status_div.empty();

    var first_entry = true;
    var error = false;

    var plot_tooltip_resources = {
        'formatter': function () {
            var o = this.point.options;

            return '<b>' + this.series.name + '</b><br>' +
                'Average: ' + this.y + '<br>' +
                'Minimum: ' + o.min + '<br>' +
                'Maximum: ' + o.max + '<br>';
        }
    };

    var plot_tooltip = {
        'formatter': function () {

            return '<b>' + this.series.name + '</b> ' + this.y;
        }
    };

    var categories = {
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
            'Packets dropped: Sent'],
        "cpu": [],
        "mem": [],
        "time": [],
        "path_length": []
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
                labels: {
                    enabled: false
                }
            },
            plotOptions: {},
            tooltip: plot_tooltip_resources
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
                labels: {
                    enabled: false
                }
            },
            plotOptions: {},
            tooltip: plot_tooltip_resources
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
            plotOptions: {},
            tooltip: plot_tooltip_resources
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
            plotOptions: {},
            tooltip: plot_tooltip_resources
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
            plotOptions: {},
            tooltip: plot_tooltip
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
            xAxis: {
                labels: {
                    enabled: false
                }
            },
            plotOptions: {},
            tooltip: plot_tooltip
        }
    };
    var time_data = [];

    configuration_div.append('<li><b>Scene config:</b> ' + test_data["scene_config"] + '</li>' +
        '<li><b>Test config:</b> ' + test_data["test_config"] + '</li>' +
        '<li><b>Robot:</b> ' + test_data["robot"] + '</li>' +
        '<li><b>Planer ID:</b> ' + test_data["planer_id"] + '</li>' +
        '<li><b>Planning Method:</b> ' + test_data["planning_method"] + '</li>' +
        '<li><b>Jump threshold:</b> ' + test_data["jump_threshold"] + '</li>' +
        '<li><b>EEF_step:</b> ' + test_data["eef_step"] + '</li>');

    $.each(test_results, function (testblock_name, testblock_data) {

        if (testblock_data.hasOwnProperty("status") && testblock_data["status"] === "error") {
            status_div.append('<div class="alert alert-danger" role="alert">Planning error in testblock "' + testblock_name + '"!</div>');
            error = true;
        } else if (testblock_name === "error") {
            status_div.append('<div class="alert alert-danger" role="alert">An error occured outside monitored testblocks. Evaluation could not be finished!</div>');
            error = true;
            return false;
        }

        var test_data = {
            "cpu": [],
            "mem": [],
            "io": [],
            "network": [],
            "path_length": []
        };

        var active_class;
        if (first_entry) {
            test_details.find('.nav-tabs').empty();
            test_details.find('.tab-content').empty();
            active_class = "active";
            first_entry = false;
        } else {
            active_class = "";
        }

        test_details.find('.nav-tabs').append('<li role="presentation" class="' + active_class + '"><a href="#details_' + testblock_name + '" aria-controls="details_' + testblock_name + '" role="tab" data-toggle="tab">' + testblock_name + '</a></li>');
        test_details.find('.tab-content').append('<div role="tabpanel" class="tab-pane ' + active_class + '" id="details_' + testblock_name + '"></div>');

        $.each(testblock_data, function (metric_name, metric_data) {

            if (metric_data instanceof Object) {
                $.each(metric_data, function (node_name, node_data) {
                    if (node_data instanceof Object) {
                        $.each(node_data, function (res_name, res_data) {
                            if (res_data["min"] instanceof Array) {
                                var data = [];

                                // IO & Network
                                for (var i = 0; i < res_data["min"].length; i++) {
                                    if (res_name == "io" && i > 1) {
                                        res_data["average"][i] = round(res_data["average"][i]/1000, 3);
                                        res_data["min"][i] = round(res_data["min"][i]/1000, 3);
                                        res_data["max"][i] = round(res_data["max"][i]/1000, 3);
                                    } else if (res_name == "network" && i < 2) {
                                        res_data["average"][i] = round(res_data["average"][i]/1000, 3);
                                        res_data["min"][i] = round(res_data["min"][i]/1000, 3);
                                        res_data["max"][i] = round(res_data["max"][i]/1000, 3);
                                    }
                                    data.push({
                                        'x': i,
                                        'y': res_data["average"][i],
                                        'min': res_data["min"][i],
                                        'max': res_data["max"][i]
                                    });
                                }
                                test_data[res_name].push({
                                    'name': node_name,
                                    'data': data
                                });
                            }
                            else {
                                /// CPU & Mem
                                test_data[res_name].push({
                                    'name': node_name,
                                    'data': [{
                                        'x': 0,
                                        'y': res_data["average"],
                                        'min': res_data["min"],
                                        'max': res_data["max"]
                                    }]
                                });
                            }
                        });
                    } else {
                        // Path length
                        test_data[metric_name].push({
                            'name': node_name,
                            'data': [{
                                'x': 0,
                                'y': node_data
                            }]
                        });
                    }
                });
            } else {
                if (typeof metric_data == "number") {
                    /// Time
                    time_data.push({
                        'name': testblock_name,
                        'data': [{
                            'x': 0,
                            'y': metric_data
                        }]
                    });
                }
            }
        });

        $.each(test_data, function (metric_name, data) {
            if (data.length != 0) {
                test_details.show();

                var testblock_tab_content = test_details_tab_content.find('#details_' + testblock_name);
                testblock_tab_content.append('<div class="panel panel-info"><div class="panel-heading"></div>' +
                    '<div class="panel-body"><div id="details_' + testblock_name + '_' + metric_name + '_content" class="plot"></div></div></div>');

                plot_options[metric_name]["xAxis"]["categories"] = categories[metric_name];

                $('#details_' + testblock_name).find('#details_' + testblock_name + '_' + metric_name + '_content').highcharts({
                    chart: plot_options[metric_name]["chart"],
                    title: plot_options[metric_name]["title"],
                    xAxis: plot_options[metric_name]["xAxis"],
                    yAxis: plot_options[metric_name]["yAxis"],
                    tooltip: plot_options[metric_name]["tooltip"],
                    series: data,
                    plotOptions: plot_options[metric_name]["plotOptions"]
                });
            }
        });
    });
    if (time_data.length != 0) {
        test_details.show();

        var details_time = $('#details_time');
        details_time.empty().append('<div class="panel panel-primary"><div class="panel-heading"></div>' +
            '<div class="panel-body"><div id="details_time_content" class="plot"></div></div></div>');
        $('#details_time_content').highcharts({
            chart: plot_options["time"]["chart"],
            title: plot_options["time"]["title"],
            xAxis: plot_options["time"]["xAxis"],
            yAxis: plot_options["time"]["yAxis"],
            tooltip: plot_options["time"]["tooltip"],
            series: time_data,
            plotOptions: plot_options["time"]["plotOptions"]
        });
    }

    if (!error) {
        status_div.append('<div class="alert alert-success" role="alert">No error during evaluation!</div>');
    }
}
