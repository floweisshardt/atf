$(document).ready(function() {

    // Extending jQuery.when
    if (jQuery.when.all === undefined) {
        jQuery.when.all = function(deferreds) {
            var deferred = new jQuery.Deferred();

            $.when.apply(jQuery, deferreds).then(
                function () {
                    deferred.resolve(Array.prototype.slice.call(arguments));
                },
                function () {
                    deferred.fail(Array.prototype.slice.call(arguments));
                }
            );
            return deferred;
        }
    }

    // Load test list from storage (if available)
    if (getDataFromStorage("test_list")) {
        showTestList();
    }

    $('#button_compare').prop("disabled", true);
});

$(document).on('change', '.btn-file :file', function () {
    $('#button_compare').prop("disabled", true);

    var labels = [];
    var input = $(this),
        numFiles = input.get(0).files ? input.get(0).files.length : 1;
    for (var files = 0; files < input.get(0).files.length; files++) {
        if(input.get(0).files[parseInt(files)].name.indexOf(".json") === -1) {
            alert("Select only .json files!");
            return;
        }
        labels.push(input.get(0).files[parseInt(files)].name.split(".")[0]);
    }
    if ($.inArray("test_list", labels) === -1) {
        alert("You have to select the test_list.yaml file!");
        return;
    }
    input.trigger('fileselect', [numFiles, labels]);
});

$('.btn-file :file').on('fileselect', function (event, numFiles, labels) {
    clearStorage();
    getData("./data/", labels);
});

$('#test_list').on("click", ".btn", function (e) {
    e.preventDefault();
    var name = $(this).attr('data-name');
    drawTestDetails(name);
});

$('.nav-tabs').on("click", "a", function (e) {
    e.preventDefault();
    $(this).tab('show');
});

$('.table').find('#test_list').on("click", "input", function () {
    if ($(this).is(":checked")) {
        var test_config = $(this).parent().parent().parent().parent().find('.test_config').html();
        var scene_config = $(this).parent().parent().parent().parent().find('.scene_config').html();
        $('.table').find('#test_list .test_config').each(function() {
            if ($(this).parent().find('#button_detail').prop("disabled") == false && $(this).parent().find('input').prop("disabled") == false) {
                if ($(this).html() != test_config || $(this).parent().find('.scene_config').html() != scene_config) {
                    $(this).parent().addClass('danger');
                    $(this).parent().find('input').prop("disabled", true);
                } else {
                    $(this).parent().removeClass('danger');
                    $(this).parent().find('input').prop("disabled", false);
                }
            }
        });
    } else {
        var selected = 0;
        $(this).parent().parent().parent().parent().parent().find('input:checked').each(function () {
            selected += 1;
        });
        if (selected === 0) {
            $('.table').find('#test_list .test_config').each(function () {
                if ($(this).parent().find('#button_detail').prop("disabled") == false && ($(this).parent().find('input').prop("disabled") == false || $(this).parent().hasClass('warning') == false)) {
                    $(this).parent().removeClass('danger');
                    $(this).parent().find('input').prop("disabled", false);
                }
            });
        }
    }
    var checked = 0;
    $(this).parent().parent().parent().parent().parent().find('input:checked').each(function () {
        checked += 1;
    });

    if (checked > 1) {
        $('#button_compare').prop("disabled", false);
    } else {
        $('#button_compare').prop("disabled", true);
    }
});

$('#compare_tests').find('#weight_control .btn-group').on("click", "button", function (e) {
    if ($(this).hasClass('active')) {
        $(this).removeClass('active');
    } else {
        $(this). addClass('active');
    }
});

$(document).on("click", "#button_compare", function () {
    var tests = [];
    var table = $('.table');

    table.find('#test_list input:checked').each(function () {
        tests.push($(this).val());
        $(this).prop("checked", false);
    });

    table.find('#test_list .test_config').each(function () {
        if ($(this).parent().find('#button_detail').prop("disabled") == false && ($(this).parent().find('input').prop("disabled") == false || $(this).parent().hasClass('warning') == false)) {
            $(this).parent().removeClass('danger');
            $(this).parent().find('input').prop("disabled", false);
        }
    });
    compareTests(tests);
});
