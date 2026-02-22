# -*- mode: ruby -*-
# vi: set ft=ruby :
#
# Vagrant configuration file
#
# References:
#
# * Vagrantfile | Vagrant | HashiCorp Developer
#   https://developer.hashicorp.com/vagrant/docs/vagrantfile
#
# This work is based on "The common Vagrantfile templates" project:
# https://gitlab.com/the-common/vagrantfile-templates
#
# Copyright 2026 林博仁(Buo-ren Lin) <buo.ren.lin@gmail.com>
# SPDX-License-Identifier: CC-BY-SA-4.0+

Vagrant.configure("2") do |config|
  # Get the directory name containing this Vagrantfile to use as VM name prefix
  project_name = File.basename(Dir.pwd)

  # Find more Vagrant boxes at:
  # https://portal.cloud.hashicorp.com/vagrant/discover?architectures=amd64&providers=virtualbox&query=bento
  config.vm.box = "bento/ubuntu-24.04"

  # Configure synced folders for ease access to project files
  config.vm.synced_folder ".", "/home/vagrant/snapcraft"

  config.vm.provider :virtualbox do |v|
    v.cpus = 4
    v.memory = 8192

    # Use Virt-IO network adapter to improve networking performance
    v.default_nic_type = "virtio"
  end

  config.vm.define "dev" do |dev|
    dev.vm.hostname = "dev"

    dev.vm.provider :virtualbox do |v|
      v.name = "#{project_name} - Development environment"
    end

    dev.vm.provision "shell", inline: "/home/vagrant/snapcraft/dev-assets/deploy-development-environment.sh"
    dev.vm.provision "shell", privileged: false, inline: "sudo apt install -y make"
  end
end
