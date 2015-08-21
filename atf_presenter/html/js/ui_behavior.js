$('.sidebar > ul.nav li a').click(function(e) {
    $('.sr-only').remove();
    var $this = $(this);
    $this.parent().siblings().removeClass('active').end().addClass('active');
    $this.append('<span class="sr-only">(current)</span>');
    e.preventDefault();
});

$('#test_list').on("click", ".btn", function(e){
    e.preventDefault(); // preventing default click action
    var name = $(this).attr('data-name');
    getTestDetails(name);
});
