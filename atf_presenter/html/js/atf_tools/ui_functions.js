$(document).ready(function () {

  document.getElementById('file_input').addEventListener('change', handleFileSelect, false);
  document.getElementById('file_input_dropbox').addEventListener('click', handleFileSelectDropbox, false);

  $('#button_compare').prop('disabled', true);
  $('#abort_connection').prop('disabled', true);
  $('#refresh_status_list').prop('disabled', true);
  $('#remove_test').prop('disabled', true);

  var compare_test_option = $('#compare_test_option');

  compare_test_option.find('#select_test_config').selectpicker({
    style: 'btn-primary',
    size: 4
  });
  compare_test_option.find('#select_scene_config').selectpicker({
    style: 'btn-primary',
    size: 4
  });

  compare_test_option.find('.selectpicker').prop('disabled', true).selectpicker('refresh);');

  // Load test list from storage (if available)
  if (FileStorage.readData('test_list')) {
    TestList.metrics = FileStorage.readData('metrics');
    TestList.show();
  }
});

$('#test_list').on('click', '.btn', function (e) {
  e.preventDefault();
  TestList.showDetails($(this).attr('data-name'));
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
  $('#text_selection').find('span').html(checked);
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

  var weight_control_buttons = $('#compare_tests').find('#weight_control .weight_control_button');

  if (!$(this).hasClass('active')) {
    var active_buttons = $(this).parent().parent().parent().find('.active');
    var active = active_buttons.length;

    if (active === (weight_control_buttons.length - 1)) {
      active_buttons.each(function () {
        $(this).removeClass('active');
        weight_category = $(this).val();
        weight_input = $(this).parent().parent().find('.weight_control_value');
        if (weight_input.val() === '0') {
          weight_factor = 1;
        } else {
          weight_factor = 1 / weight_input.val();
        }
        weight_input.prop('disabled', false);
        TestComparison.changeWeight(weight_category, weight_factor);
      });
    }
    $(this).addClass('active');
    weight_category = $(this).val();
    weight_input = $(this).parent().parent().find('.weight_control_value');
    weight_factor = weight_input.val();
    weight_input.prop('disabled', true);
    TestComparison.changeWeight(weight_category, weight_factor);
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
    TestComparison.changeWeight(weight_category, weight_factor);
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
  $('#text_selection').find('span').html('0');
  TestComparison.compareTests(tests);
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
    $('#text_selection').find('span').html(checked);
    if (checked > 1) {
      $('#button_compare').prop('disabled', false);
    } else {
      $('#button_compare').prop('disabled', true);
    }
  }
);

$(document).on('click', '#connect_to_rosmaster', function () {

  var connect_status = $('#connect_status');
  var status_panel = $('#test_status_panel');
  var test_counter = $('#test_counter');
  var test_status_list = $('#test_status_list');

  connect_status.removeClass('alert-success')
    .removeClass('alert-danger')
    .removeClass('alert-warning')
    .empty()
    .show()
    .addClass('alert-warning')
    .append('Connecting...');
  status_panel.hide();
  test_counter.empty();
  test_status_list.empty();

  ros.connection_timeout = 0;
  ros.connectToServer();
});

$(document).on('click', '#abort_connection', function () {
  ros.connection_timeout = ros.connection_attempts;
});

$(document).on('click', '#refresh_status_list', function () {
  ros.callService();
});

$('.sidebar > ul.nav li a')
  .mouseenter(function (e) {
    $(this).parent().addClass('active');
    e.preventDefault();
  })
  .mouseleave(function (e) {
    $(this).parent().removeClass('active');
    e.preventDefault();
  });
