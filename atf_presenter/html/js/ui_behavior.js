$(document).ready(function () {

  // Extending jQuery.when
  if (jQuery.when.all === undefined) {
    jQuery.when.all = function (deferreds) {
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

  $('#button_compare').prop('disabled', true);
  var compare_test_option = $('#compare_test_option');

  compare_test_option.find('#select_test_config').selectpicker({
    style: 'btn-primary',
    size: 4
  });
  compare_test_option.find('#select_scene_config').selectpicker({
    style: 'btn-primary',
    size: 4
  });

  compare_test_option.find('.selectpicker').prop('disabled', true).selectpicker('refresh);')

  // Load test list from storage (if available)
  if (getDataFromStorage('test_list')) {
    showTestList();
  }
});

$(document).on('change', '.btn-file :file', function () {
  $('#button_compare').prop('disabled', true);
  var labels = [];
  var input = $(this);
  var numFiles = input.get(0).files ? input.get(0).files.length : 1;
  for (var files = 0; files < input.get(0).files.length; files++) {
    if (input.get(0).files[files].name.indexOf('.json') === -1) {
      bootbox.alert('Select only .json files!');
      return;
    }
    labels.push(input.get(0).files[files].name.split('.')[0]);
  }
  if ($.inArray('test_list', labels) === -1) {
    bootbox.alert('You have to select the test_list.yaml file!');
    return;
  }
  input.trigger('fileselect', [numFiles, labels]);
});

$('.btn-file :file').on('fileselect', function (event, numFiles, labels) {
  clearStorage();
  getData('./data/', labels);
});

$('#test_list').on('click', '.btn', function (e) {
  e.preventDefault();
  var name = $(this).attr('data-name');
  drawTestDetails(name);
});

$('.nav-tabs').on('click', 'a', function (e) {
  e.preventDefault();
  $(this).tab('show');
});

$('.table').find('#test_list').on('click', 'input', function () {
  if ($(this).is(':checked')) {
    var test_config = $(this).parent().parent().parent().parent().find('.test_config').html();
    var scene_config = $(this).parent().parent().parent().parent().find('.scene_config').html();

    $('.table').find('#test_list .test_config').each(function () {
      if ($(this).parent().find('#button_detail').prop('disabled') === false && $(this).parent().find('input').prop('disabled') === false) {
        if ($(this).html() != test_config || $(this).parent().find('.scene_config').html() != scene_config) {
          $(this).parent().addClass('danger').find('input').prop('disabled', true);
        } else {
          $(this).parent().removeClass('danger').find('input').prop('disabled', false);
        }
      }
    });
  } else {
    var selected = $(this).parent().parent().parent().parent().parent().find('input:checked').length;
    if (selected === 0) {
      $('.table').find('#test_list .test_config').each(function () {
        if ($(this).parent().find('#button_detail').prop('disabled') === false && ($(this).parent().find('input').prop('disabled') === false || $(this).parent().hasClass('warning') === false)) {
          $(this).parent().removeClass('danger').find('input').prop('disabled', false);
        }
      });
    }
  }
  var checked = $(this).parent().parent().parent().parent().parent().find('input:checked').length;
  if (checked > 1) {
    $('#button_compare').prop('disabled', false);
  } else {
    $('#button_compare').prop('disabled', true);
  }
});

$('#compare_tests').find('#weight_control').on('click', '.weight_control_button', function () {
  var weight_input;
  var weight_factor;
  var weight_category;

  if (!$(this).hasClass('active')) {
    var active_buttons = $(this).parent().parent().parent().find('.active');
    var active = active_buttons.length;

    if (active === 2) {
      active_buttons.each(function () {
        $(this).removeClass('active');
        weight_category = $(this).val();
        weight_input = $(this).parent().parent().find('.weight_control_value');
        weight_factor = 1 / weight_input.val();
        weight_input.prop('disabled', false);
        changeWeight(weight_category, weight_factor);
      });
    }
    $(this).addClass('active');
    weight_category = $(this).val();
    weight_input = $(this).parent().parent().find('.weight_control_value');
    weight_factor = weight_input.val();
    weight_input.prop('disabled', true);
    changeWeight(weight_category, weight_factor);
  } else {
    $(this).removeClass('active');
    weight_category = $(this).val();
    weight_input = $(this).parent().parent().find('.weight_control_value');

    if (weight_input.val() === '0') {
      weight_factor = 1;
    } else {
      weight_factor = 1 / weight_input.val();
    }
    weight_input.prop('disabled', false);
    changeWeight(weight_category, weight_factor);
  }
});

$(document).on('click', '#button_compare', function () {
  var tests = [];
  var table = $('.table');
  var compare_test_option = $('#compare_test_option');
  var compare_test_option_select_test = compare_test_option.find('#select_test_config');
  var compare_test_option_select_scene = compare_test_option.find('#select_scene_config');
  var compare_tests_weight_control = $('#compare_tests').find('#weight_control');

  table.find('#test_list input:checked').each(function () {
    tests.push($(this).val());
    $(this).prop('checked', false);
  });

  table.find('#test_list .test_config').each(function () {
    if ($(this).parent().find('#button_detail').prop('disabled') === false && ($(this).parent().find('input').prop('disabled') === false || $(this).parent().hasClass('warning') === false)) {
      $(this).parent().removeClass('danger').find('input').prop('disabled', false);
    }
  });

  compare_tests_weight_control
    .find('.weight_control_button').each(function () {
      $(this).removeClass('active');
    })
    .find('.weight_control_value').each(function () {
      $(this).prop('disabled', false);
    });

  $(this).prop('disabled', true);
  compare_test_option_select_test.val('').selectpicker('refresh');
  compare_test_option_select_scene.val('').prop('disabled', true).selectpicker('refresh');
  compareTests(tests);
});

$('#compare_test_option')
  .on('change', '#select_test_config', function () {
    var config_selected = $(this).val();
    if (config_selected != 'None') {
      var test_found = 0;
      $('#test_list_content').find('.table #test_list .test_config').each(function () {
        if ($(this).parent().find('#button_detail').prop('disabled') === false && ($(this).parent().find('input').prop('disabled') === false || $(this).parent().hasClass('warning') === false)) {
          if ($(this).html() === config_selected) {
            test_found++;
          }
        }
      });
      if (test_found > 1) {
        $('#select_scene_config').prop('disabled', false).selectpicker('refresh');
      } else {
        $('#select_scene_config').prop('disabled', true).selectpicker('refresh');
      }
    } else {
      $('#select_scene_config').prop('disabled', true).selectpicker('refresh');
    }
  }
)
  .on('change', '#select_scene_config', function () {
    var config_selected = $('#select_test_config').val();
    var checked = 0;
    var scene_config = $(this).val();
    $('#test_list_content').find('.table #test_list .test_config').each(function () {
      if ($(this).parent().find('#button_detail').prop('disabled') === false && ($(this).parent().find('input').prop('disabled') === false || $(this).parent().hasClass('warning') === false)) {
        if ($(this).html() === config_selected && $(this).parent().find('.scene_config').html() === scene_config) {
          $(this).parent().removeClass('danger').find('input').prop('checked', true).prop('disabled', false);
          checked++;
        } else if (scene_config === 'None') {
          $(this).parent().removeClass('danger').find('input').prop('checked', false).prop('disabled', false);
        } else {
          $(this).parent().addClass('danger').find('input').prop('checked', false).prop('disabled', true);
        }
      }
    });
    if (checked > 1) {
      $('#button_compare').prop('disabled', false);
    } else {
      $('#button_compare').prop('disabled', true);
    }
  }
);