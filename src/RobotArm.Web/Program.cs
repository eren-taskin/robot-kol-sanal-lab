using Microsoft.AspNetCore.Components.Web;
using Microsoft.AspNetCore.Components.WebAssembly.Hosting;
using RobotArm.Web;
using RobotArm.Web.Models;

var builder = WebAssemblyHostBuilder.CreateDefault(args);
builder.RootComponents.Add<App>("#app");
builder.RootComponents.Add<HeadOutlet>("head::after");

builder.Services.AddScoped(sp => new HttpClient { BaseAddress = new Uri(builder.HostEnvironment.BaseAddress) });
// T10: the setup and the compared runs survive a visit to the robustness page.
builder.Services.AddScoped<LabState>();
builder.Services.AddScoped<RunHistory>();

await builder.Build().RunAsync();
