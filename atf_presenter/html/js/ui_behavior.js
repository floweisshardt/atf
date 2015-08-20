$('.sidebar > ul.nav li a').click(function(e) {
    $('.sr-only').remove();
    var $this = $(this);
    $this.parent().siblings().removeClass('active').end().addClass('active');
    $this.append('<span class="sr-only">(current)</span>');
    e.preventDefault();
});

function showTestDetails(test_name) {
    alert(test_name);
}