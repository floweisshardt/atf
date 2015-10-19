var ros = {
  url: '',
  roscore: {},
  service_server: {},
  service_request: {},
  test_update: {},
  tests_total: 0,
  connection_timeout: 0,
  connection_attempts: 10,
  connection_lost: false,
  connectToServer: function () {
    var this_class = this;
    var connect_status = $('#connect_status');
    var connect_status_label = $('#connect_status_label');
    var service_status = $('#service_status');
    var abort_connection = $('#abort_connection');
    var refresh_status_list = $('#refresh_status_list');

    this.url = 'ws://' + $('#ros_master_ip').val() + ':9090';
    abort_connection.prop('disabled', false);

    this.roscore = new ROSLIB.Ros({
      url: this.url
    });

    this.service_server = new ROSLIB.Service({
      ros: this_class.roscore,
      name: 'atf/get_test_status',
      serviceType: 'atf_status_server/GetTestStatus'
    });

    this.service_request = new ROSLIB.ServiceRequest({
      request: true
    });

    this.roscore.on('connection', function () {
      console.log('Connected to websocket server.');
      connect_status.removeClass('alert-success')
        .removeClass('alert-danger')
        .removeClass('alert-warning')
        .empty()
        .show()
        .addClass('alert-success')
        .append('Connection established!');

      connect_status_label.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-success');

      service_status.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-danger');

      abort_connection.prop('disabled', true);

      this_class.connection_timeout = 0;
      this_class.connection_lost = true;
      this_class.callService();
    });

    this.roscore.on('error', function (error) {
      console.log('Error connecting to websocket server: ', error);
    });

    this.roscore.on('close', function () {
      console.log('Connection to websocket server closed.');

      var msg = '';
      if (this_class.connection_lost) msg = 'Connection lost!';
      else msg = 'Connection could not be established!';

      connect_status.removeClass('alert-success')
        .removeClass('alert-danger')
        .removeClass('alert-warning')
        .empty()
        .show()
        .addClass('alert-danger')
        .append(msg + ' Reconnecting... (' + parseInt(this_class.connection_timeout + 1) + ')');

      connect_status_label.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-danger');

      service_status.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-danger');

      refresh_status_list.prop('disabled', true);

      if (this_class.connection_timeout === this_class.connection_attempts) {
        connect_status.removeClass('alert-success')
          .removeClass('alert-danger')
          .removeClass('alert-warning')
          .empty()
          .show()
          .addClass('alert-danger')
          .append(msg);

        abort_connection.prop('disabled', true);

        return true;
      } else this_class.connection_timeout++;

      this_class.connectToServer();
    });
  },
  callService: function () {
    var this_class = this;
    var service_status = $('#service_status');
    this.test_update = {};

    this.service_server.callService(this.request, function (res) {
      service_status.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-success');
      $.each(res['status'], function (index, data) {
        this_class.tests_total = data['total'];
        this_class.test_update[data['test_name']] = {
          'status': [data['status_recording'], data['status_analysing']],
          'testblock': data['testblock']
        };
      });
      this_class.buildStatusList();
    }, function () {
      service_status.removeClass('label-success')
        .removeClass('label-danger')
        .addClass('label-danger');
      $('#refresh_status_list').prop('disabled', true);
      this_class.callService();
    });
  },
  buildStatusList: function () {
    var status_panel = $('#test_status_panel');
    var test_counter = $('#test_counter');
    var test_status_list = $('#test_status_list');
    var refresh_status_list = $('#refresh_status_list');

    status_panel.show();
    test_counter.empty().show();
    test_status_list.empty();
    refresh_status_list.prop('disabled', false);

    var finished = 0;

    $.each(this.test_update, function (name, data) {
      var status_record, status_analyse = '';
      if (data['status'][0] === 0) status_record = '<span class="glyphicon glyphicon-hourglass" title="Waiting" aria-hidden="true"></span><span class="sr-only">Waiting</span>';
      else if (data['status'][0] === 1) status_record = '<span class="glyphicon glyphicon-cog" title="Running" aria-hidden="true"></span><span class="sr-only">Running</span>';
      else if (data['status'][0] === 3) status_record = '<span class="glyphicon glyphicon-ok" title="Finished" aria-hidden="true"></span><span class="sr-only">Finished</span>';

      if (data['status'][1] === 0) status_analyse = '<span class="glyphicon glyphicon-hourglass" title="Waiting" aria-hidden="true"></span><span class="sr-only">Waiting</span>';
      else {
        $.each(data['testblock'], function (index, testblock_data) {
          if (testblock_data['status'] === 0) status_analyse += '<span class="glyphicon glyphicon-hourglass" title="' + testblock_data['name'] + ': Waiting" aria-hidden="true"></span><span class="sr-only">' + testblock_data['name'] + ': Waiting</span>';
          else if (testblock_data['status'] === 1) status_analyse += '<span class="glyphicon glyphicon-cog" title="' + testblock_data['name'] + ': Running" aria-hidden="true"></span><span class="sr-only">' + testblock_data['name'] + ': Running</span>';
          else if (testblock_data['status'] === 2) status_analyse += '<span class="glyphicon glyphicon-pause" title="' + testblock_data['name'] + ': Paused" aria-hidden="true"></span><span class="sr-only">' + testblock_data['name'] + ': Paused</span>';
          else if (testblock_data['status'] === 3) status_analyse += '<span class="glyphicon glyphicon-ok" title="' + testblock_data['name'] + ': Finished" aria-hidden="true"></span><span class="sr-only">' + testblock_data['name'] + ': Finished</span>';
          else if (testblock_data['status'] === 4) status_analyse +='<span class="glyphicon glyphicon-remove" title="' + testblock_data['name'] + ': Error" aria-hidden="true"></span><span class="sr-only">' + testblock_data['name'] + ': Error</span>';
        });
      }

      test_status_list.append('<tr><td>' + name + '</td>' +
        '<td>' + status_record + '</td>' +
        '<td>' + status_analyse + '</td></tr>');
      if ((data['status'][0] === 3 && data['status'][1]) === 3) finished++;
    });

    test_counter.append('<b>Tests finished:</b> ' + finished + ' / ' + this.tests_total);
  }
};