pipeline {
  agent {
  // Equivalent to "docker build -f ./.devcontainer/Dockerfile --build-arg name=value ./sr
  dockerfile { // dockerfile parameter
    filename             './.devcontainer/Dockerfile' // filename argument and value
    dir                  './'
    //additionalBuildArgs  '--build-arg name=value'
    args                 '-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp:/tmp -u root ' // apparently SELinux doesn't let us use "ros:ros" as a user. https://stackoverflow.com/questions/58346984/how-to-fix-process-apparently-never-started-in-error-in-jenkins-pipeline
  }
}

  // Poll SCM
  triggers {
    //Hourly polling
    pollSCM('H */1 * * *')
  }
  stages {

    stage('Building') {
      steps {
        sh '''
        printenv ;
        pwd;
        id;
        SIM_ENV=CICD ./start.sh build
        '''
      }
    }
    stage('Pre-commit hooks') {
      steps {
          sh '''
          git config --global --add safe.directory '*' && pre-commit run --all-files
          '''
      }
    }
    stage('Running Simulation') {
      steps {
        sh '''
        SIM_ENV=CICD ./start.sh sim
        '''
      }
    }
  }
}
