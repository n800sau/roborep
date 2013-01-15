from django.conf.urls import patterns, include, url
import views

# Uncomment the next two lines to enable the admin:
# from django.contrib import admin
# admin.autodiscover()

urlpatterns = patterns('',

	url('^$', views.home),
	url('^monitor/$', views.monitor),
	url('^histogram/$', views.histogram),
	url('^move_right/$', views.move_right),
	url('^move_left/$', views.move_left),
	url('^move_up/$', views.move_up),
	url('^move_down/$', views.move_down),
	url('^make_image/$', views.make_image),
	url('^get_image/$', views.get_image),
	url('^cam_values/$', views.cam_values),

	# Examples:
	# url(r'^$', 'djface.views.home', name='home'),
	# url(r'^djface/', include('djface.foo.urls')),

	# Uncomment the admin/doc line below to enable admin documentation:
	# url(r'^admin/doc/', include('django.contrib.admindocs.urls')),

	# Uncomment the next line to enable the admin:
	# url(r'^admin/', include(admin.site.urls)),
)
