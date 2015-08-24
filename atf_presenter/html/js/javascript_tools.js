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
    document.getElementById("test_list").innerHTML = "";
    var number = 1;
    test_list.forEach(function(test_data) {
        var test_name = Object.keys(test_data)[0];
        var test_name_full = test_name.split("_");
        document.getElementById("test_list").innerHTML += "<tr><td>" + (number) + "</td><td>" + test_name + "</td><td>Testsuite " + test_name_full[0].replace(/^\D+/g, '') + "</td><td>Test " + test_name_full[1].replace(/^\D+/g, "") + "</td><td>" + test_data[test_name].test_config + "</td><td><button type='button' class='btn btn-default' data-target='#test_detail' data-toggle='modal' data-name='" + test_name + "'>Details</button></td>";
        number++;
    });
}

function drawTestDetails(test_name) {
    var test_detail = $('#test_detail');
    var test_name_split = test_name.split("_");
    test_detail.find('.modal-title').html("Details Testsuite " + test_name_split[0].replace(/^\D+/g, "") + " - Test " + test_name_split[1].replace(/^\D+/g, ""));

    // Get test data
    var test_data = getDataFromStorage(test_name);

    // Get test list
    var test_list = getDataFromStorage("test_list");



    $.each(test_data, function(index, value) {
        console.log(index, value);
    });
    //plotData('res_cpu', [[1], [2], [3]]);
    //test_detail.find('.modal-body p').html(JSON.stringify(sessionStorage.getItem(test_name)));

    $('#res_cpu').highcharts({
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
        tooltip: {
            formatter: function () {
                var o = this.point.options;

                return '<b>' + this.series.name + '</b><br>' +
                    'Average: ' + this.y + '<br>' +
                    'Min: ' + o.min + '<br>' +
                    'Max: ' + o.max + '<br>';
            }
        },
        series: [{
            name: 'move_group',
            data: [{
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }, {
            name: 'rostest',
            data: [{
                x: 1,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }]
        }, {
            name: 'security',
            data: [{
                x: 1,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }]
        }]
    });

    $('#res_mem').highcharts({
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
        tooltip: {
            formatter: function () {
                var o = this.point.options;

                return '<b>' + this.series.name + '</b><br>' +
                    'Average: ' + this.y + '<br>' +
                    'Min: ' + o.min + '<br>' +
                    'Max: ' + o.max + '<br>';
            }
        },
        series: [{
            name: 'move_group',
            data: [{
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }, {
            name: 'rostest',
            data: [{
                x: 1,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }]
        }, {
            name: 'security',
            data: [{
                x: 1,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }]
        }]
    });
    $('#res_io').highcharts({
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
        tooltip: {
            formatter: function () {
                var o = this.point.options;

                return '<b>' + this.series.name + '</b><br>' +
                    'Average: ' + this.y + '<br>' +
                    'Min: ' + o.min + '<br>' +
                    'Max: ' + o.max + '<br>';
            }
        },
        series: [{
            name: 'move_group',
            data: [{
                x: 0,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 2,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 3,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }, {
            name: 'rostest',
            data: [{
                x: 0,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }, {
                x: 1,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }, {
                x: 2,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }, {
                x: 3,
                y: 80.01,
                min: 20.0,
                max: 120.43
            }]
        }, {
            name: 'security',
            data: [{
                x: 0,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }, {
                x: 1,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }, {
                x: 2,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }, {
                x: 3,
                y: 50.03,
                min: 2.05,
                max: 98.44
            }]
        }]
    });
    $('#res_network').highcharts({
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
        tooltip: {
            formatter: function () {
                var o = this.point.options;

                return '<b>' + this.series.name + '</b><br>' +
                    'Average: ' + this.y + '<br>' +
                    'Min: ' + o.min + '<br>' +
                    'Max: ' + o.max + '<br>';
            }
        },
        series: [{
            name: 'move_group',
            data: [{
                x: 0,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 2,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 3,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 4,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 5,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 6,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 7,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 8,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }, {
            name: 'rostest',
            data: [{
                x: 0,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 2,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 3,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 4,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 5,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 6,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 7,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 8,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }, {
            name: 'security',
            data: [{
                x: 0,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 1,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 2,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 3,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 4,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 5,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 6,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 7,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }, {
                x: 8,
                y: 99.94,
                min: 8.0,
                max: 111.4
            }]
        }]
    });

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
            formatter: function () {
                var o = this.point.options;

                return '<b>' + this.series.name + '</b><br>' +
                    'Time: ' + this.y;
            }
        },
        plotOptions: {
            pie: {
                allowPointSelect: true,
                cursor: 'pointer',
                dataLabels: {
                    enabled: false
                },
                showInLegend: true
            }
        },
        series: [{
            name: "Time",
            colorByPoint: true,
            data: [{
                name: "execution_1",
                y: 9.272
            }, {
                name: "execution_2",
                y: 64.581,
                sliced: true,
                selected: true
            }, {
                name: "execution_3",
                y: 10.916
            }, {
                name: "execution_all",
                y: 106.139
            }, {
                name: "planning_1",
                y: 4.413
            }, {
                name: "planning_2",
                y: 17.329
            }, {
                name: "planning_3",
                y: 3.458
            }, {
                name: "planning_all",
                y: 99.581
            }]
        }]
    });
}

$(document).ready( function() {
    $('.btn-file :file').on('fileselect', function(event, numFiles, labels) {
        for (var x = 0; x < labels.length; x++) {
            if (labels[x] == "test_list") {
                getJSON("./data/", labels[x], drawTestList);
            } else {
                getJSON("./data/", labels[x]);
            }
        }
        $('#test_list_content').show();
    });

    if (sessionStorage.getItem("test_list") != null) {
        drawTestList();
        $('#test_list_content').show();
    }
});
