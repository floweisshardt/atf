var ros = {
  //url: 'ws://10.0.1.215:9090',
  url: 'ws://localhost:9090',
  roscore: {},
  service_server: {},
  service_request: {},
  test_update: {},
  tests_total: 0,
  connection_timeout: 0,
  connectToServer: function () {
    var this_class = this;

    this.roscore = new ROSLIB.Ros({
      url: this.url
    });

    this.service_server = new ROSLIB.Service({
      ros: this_class.roscore,
      name: 'atf/get_test_status',
      serviceType: 'atf_server/GetTestStatus'
    });

    this.service_request = new ROSLIB.ServiceRequest({
      request: true
    });

    this.roscore.on('connection', function () {
      console.log('Connected to websocket server.');
      this_class.connection_timeout = 0;
      this_class.callService();
    });

    this.roscore.on('error', function (error) {
      console.log('Error connecting to websocket server: ', error);
    });

    this.roscore.on('close', function () {
      console.log('Connection to websocket server closed.');
      if (this_class.connection_timeout === 5) {
        return true;
      } else this_class.connection_timeout++;
      this_class.connectToServer();
    });
  },
  callService: function () {
    var this_class = this;
    this.test_update = {};
    this.service_server.callService(this.request, function (res) {
      $.each(res.status, function (index, data) {
        this_class.tests_total = data.total;
        this_class.test_update[data.test_name] = [data.status_recording, data.status_analysing];
      });
      console.log(this_class.test_update);
    });
  }
};