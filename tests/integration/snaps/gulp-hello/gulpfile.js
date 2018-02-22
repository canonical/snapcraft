// grab our gulp packages
var gulp  = require('gulp'),
    gutil = require('gulp-util');

gulp.task('install', function() {
  gutil.log('Installing hello world shell script')
  return gulp.src('hello-world').pipe(gulp.dest('../install'));
});
