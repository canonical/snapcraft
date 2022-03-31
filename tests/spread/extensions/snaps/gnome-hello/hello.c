#include <gtk/gtk.h>

// Not called.
void activate(GtkApplication *app, gpointer user_data)
{
	g_print("ACTIVATED");
}

int main (int argc, char **argv)
{
	void *app = gtk_application_new("io.snapcraft.gtk-test", G_APPLICATION_FLAGS_NONE);
	g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);

	// Requires display.
	//int status = g_application_run(G_APPLICATION(app), argc, argv);

	g_print("hello world\n");

	g_object_unref(app);
	return 0;
}
